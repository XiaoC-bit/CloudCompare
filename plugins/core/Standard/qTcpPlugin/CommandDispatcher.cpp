// CommandDispatcher.cpp
#include "CommandDispatcher.h"
#include "CcTcpServer.h"

#include <FileIOFilter.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <cc2DViewportObject.h>
#include <QFileInfo>
#include <ccPolyline.h>
#include <ccMesh.h>
#include <ManualSegmentationTools.h>

CommandDispatcher::CommandDispatcher(ccMainAppInterface* app, CcTcpServer* server, QObject* parent)
    : QObject(parent)
    , m_app(app)
    , m_server(server)
{
}

void CommandDispatcher::dispatch(QJsonObject cmd, QTcpSocket* socket)
{
	QString action = cmd["action"].toString();

	if (action == "load")
		handleLoad(cmd["params"].toObject(), socket);
	else if (action == "filter")
		handleFilter(cmd["params"].toObject(), socket);
	else if (action == "icp")
		handleICP(cmd["params"].toObject(), socket);
	else if (action == "camera")
		handleCamera(cmd["params"].toObject(), socket);
	else if (action == "applyViewport")
		handleApplyViewport(cmd["params"].toObject(), socket);
	else if (action == "segment")
	{
		handleSegment(cmd["params"].toObject(), socket);
	}
	else
	{
		m_app->dispToConsole("[TcpPlugin] Unknown action: " + action, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Unknown action: " + action);
	}
}

void CommandDispatcher::handleLoad(const QJsonObject& params, QTcpSocket* socket)
{
	QString path = params["path"].toString();
	if (path.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Empty path");
		return;
	}

	QString modelName = params["name"].toString();

	FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(QFileInfo(path).suffix());
	if (!filter)
	{
		m_app->dispToConsole("[TcpPlugin] Unsupported file format", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Unsupported file format");
		return;
	}
	
	ccHObject* container = new ccHObject();
	auto result = filter->loadFile(path, *container, FileIOFilter::LoadParameters());
	if (result == CC_FERR_NO_ERROR)
	{
		if (!modelName.isEmpty())
		{
			container->setName(modelName);
		}
		
		m_app->addToDB(container);
		m_app->dispToConsole("[TcpPlugin] Loaded: " + path);
		if (m_server)
			m_server->sendResponse(socket, true, "Loaded: " + path);
	}
	else
	{
		m_app->dispToConsole("[TcpPlugin] Failed to load: " + path, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Failed to load: " + path);
		delete container;
	}
}

void CommandDispatcher::handleCamera(const QJsonObject& params, QTcpSocket* socket)
{
	auto* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "No active GL window");
		return;
	}

	if (params.contains("viewPreset"))
	{
		QString preset = params["viewPreset"].toString();
		if (preset == "top")
			glWindow->setView(CC_TOP_VIEW);
		else if (preset == "front")
			glWindow->setView(CC_FRONT_VIEW);
		else if (preset == "iso")
			glWindow->setView(CC_ISO_VIEW_1);
		glWindow->redraw();
		if (m_server)
			m_server->sendResponse(socket, true, "Camera view set to: " + preset);
	}
	else
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing viewPreset parameter");
	}
}

void CommandDispatcher::handleFilter(const QJsonObject& params, QTcpSocket* socket)
{
	if (m_server)
		m_server->sendResponse(socket, false, "Filter not implemented");
}

void CommandDispatcher::handleICP(const QJsonObject& params, QTcpSocket* socket)
{
	if (m_server)
		m_server->sendResponse(socket, false, "ICP not implemented");
}

void CommandDispatcher::handleApplyViewport(const QJsonObject& params, QTcpSocket* socket)
{
	QString name = params["name"].toString();
	if (name.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Empty name parameter");
		return;
	}

	// Get the active GL window
	ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "No active GL window");
		return;
	}

	// Search for the root object in the database
	ccHObject* rootObject = nullptr;
	ccHObject* dbRoot = m_app->dbRootObject();
	if (dbRoot)
	{
		
		for (int i = 0; i < dbRoot->getChildrenNumber(); ++i)
		{
			ccHObject* child = dbRoot->getChild(i);
			if (child)
			{
				if (child->getName() == name)
				{
					rootObject = child->getChild(0);
					break;
				}
			}
		}
	}

	if (!rootObject)
	{
		m_app->dispToConsole("[TcpPlugin] Object not found: " + name, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Object not found: " + name);
		return;
	}

	cc2DViewportObject* viewportObject = nullptr;
	for (int i = 0; i < rootObject->getChildrenNumber(); ++i)
	{
		ccHObject* obj = rootObject->getChild(i);
		if (obj)
		{
			if (obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT))
			{
				viewportObject = static_cast<cc2DViewportObject*>(obj);
				break;
			}
		}
	}

	if (!viewportObject)
	{
		m_app->dispToConsole("[TcpPlugin] Viewport object not found in hierarchy", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Viewport object not found in hierarchy");
		return;
	}

	cc2DViewportObject* viewport = ccHObjectCaster::To2DViewportObject(viewportObject);
	assert(viewport);
	if (!viewport)
	{
		return;
	}

	glWindow->setViewportParameters(viewport->getParameters());
	glWindow->redraw();

}


void CommandDispatcher::handleSegment(const QJsonObject& params, QTcpSocket* socket)
{
	QString meshName   = params["meshName"].toString();
	QString binName    = params["binName"].toString();
	bool    keepInside = params["keepInside"].toBool(true);
	QString outputName = params["outputName"].toString();

	if (meshName.isEmpty() || binName.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing meshName or binName");
		return;
	}

	ccHObject* dbRoot = m_app->dbRootObject();
	if (!dbRoot)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "DB root is null");
		return;
	}

	std::function<ccHObject*(ccHObject*, const QString&)> findByName =
	    [&](ccHObject* obj, const QString& name) -> ccHObject*
	{
		if (obj->getName() == name)
			return obj;
		for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
		{
			ccHObject* found = findByName(obj->getChild(i), name);
			if (found)
				return found;
		}
		return nullptr;
	};

	// 查找目标 Mesh
	ccHObject* meshObj    = findByName(dbRoot, meshName)->getChild(0);
	ccMesh*    targetMesh = meshObj ? ccHObjectCaster::ToMesh(meshObj) : nullptr;
	if (!targetMesh)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Mesh not found: " + meshName);
		return;
	}

	// 查找 bin 根节点
	ccHObject* binRoot = findByName(dbRoot, binName);
	if (!binRoot)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Bin root not found: " + binName);
		return;
	}

	// 在 bin 层级里找 Polyline 和 ViewportObject
	ccPolyline*         segPoly  = nullptr;
	cc2DViewportObject* vpObject = nullptr;

	std::function<void(ccHObject*)> findInHierarchy = [&](ccHObject* obj)
	{
		if (!segPoly && obj->isKindOf(CC_TYPES::POLY_LINE))
			segPoly = static_cast<ccPolyline*>(obj);
		if (!vpObject && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT))
			vpObject = static_cast<cc2DViewportObject*>(obj);
		for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
			findInHierarchy(obj->getChild(i));
	};
	findInHierarchy(binRoot);

	if (!segPoly)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Polyline not found in bin");
		return;
	}
	if (!segPoly->isClosed())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Polyline is not closed");
		return;
	}
	if (!vpObject)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Viewport object not found in bin");
		return;
	}

	// 应用 viewport，获取相机
	ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "No active GL window");
		return;
	}

	glWindow->setViewportParameters(vpObject->getParameters());
	glWindow->redraw(true, false);

	ccGLCameraParameters camera;
	glWindow->getGLCameraParameters(camera);

	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	// ---------- 关键：将 polyline 顶点从 3D 转换到 2D 屏幕坐标 ----------
	// 对应 CC 源码 doActionUseExistingPolyline() 中的转换逻辑
	ccPointCloud* polyVertices2D = new ccPointCloud();
	ccPolyline*   segPoly2D      = new ccPolyline(polyVertices2D);
	segPoly2D->addChild(polyVertices2D);

	bool                                   mode3D   = !segPoly->is2DMode();
	CCCoreLib::GenericIndexedCloudPersist* vertices = segPoly->getAssociatedCloud();

	if (!polyVertices2D->reserve(vertices->size()) || !segPoly2D->reserve(segPoly->size()))
	{
		delete segPoly2D;
		if (m_server)
			m_server->sendResponse(socket, false, "Not enough memory for polyline conversion");
		return;
	}

	for (unsigned i = 0; i < vertices->size(); ++i)
	{
		CCVector3 P = *vertices->getPoint(i);
		if (mode3D)
		{
			CCVector3d Q2D;
			camera.project(P, Q2D);
			P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
			P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
			P.z = 0;
		}
		polyVertices2D->addPoint(P);
	}
	for (unsigned j = 0; j < segPoly->size(); ++j)
	{
		segPoly2D->addPointIndex(segPoly->getPointGlobalIndex(j));
	}
	segPoly2D->setClosed(segPoly->isClosed());

	// 检查转换后的 2D 多边形是否完全在视口内
	bool polyInsideViewport = true;
	{
		int vertexCount = static_cast<int>(segPoly2D->size());
		for (int i = 0; i < vertexCount; ++i)
		{
			const CCVector3* P2D = segPoly2D->getPoint(i);
			if (P2D->x < -half_w || P2D->x > half_w || P2D->y < -half_h || P2D->y > half_h)
			{
				polyInsideViewport = false;
				break;
			}
		}
	}

	// 获取点云
	ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(targetMesh);
	if (!cloud)
	{
		delete segPoly2D;
		if (m_server)
			m_server->sendResponse(socket, false, "Mesh has no associated point cloud");
		return;
	}

	if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray())
	{
		delete segPoly2D;
		if (m_server)
			m_server->sendResponse(socket, false, "Failed to initialize visibility array");
		return;
	}

	ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
	int                                       cloudSize       = static_cast<int>(cloud->size());

	// 遍历顶点，投影并判断是否在多边形内
	for (int i = 0; i < cloudSize; ++i)
	{
		if (visibilityArray[i] == CCCoreLib::POINT_VISIBLE)
		{
			const CCVector3* P3D = cloud->getPoint(i);

			CCVector3d Q2D;
			bool       pointInFrustum = false;
			camera.project(*P3D, Q2D, &pointInFrustum);

			bool pointInside = false;
			if (pointInFrustum || !polyInsideViewport)
			{
				CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
				              static_cast<PointCoordinateType>(Q2D.y - half_h));
				pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, segPoly2D);
			}

			visibilityArray[i] = (keepInside != pointInside
			                          ? CCCoreLib::POINT_HIDDEN
			                          : CCCoreLib::POINT_VISIBLE);
		}
	}

	// 生成新 Mesh
	ccMesh* segmentedMesh = targetMesh->createNewMeshFromSelection(false);

	// 恢复原始点云可见性
	cloud->resetVisibilityArray();

	// 释放临时 2D polyline
	delete segPoly2D;

	if (!segmentedMesh || segmentedMesh->size() == 0)
	{
		delete segmentedMesh;
		if (m_server)
			m_server->sendResponse(socket, false, "Segmentation result is empty");
		return;
	}

	segmentedMesh->setName(outputName.isEmpty() ? targetMesh->getName() + "_segmented" : outputName);
	m_app->addToDB(segmentedMesh);
	m_app->dispToConsole("[TcpPlugin] Segment OK: " + segmentedMesh->getName());
	if (m_server)
		m_server->sendResponse(socket, true, "Segmentation completed: " + segmentedMesh->getName());
}
