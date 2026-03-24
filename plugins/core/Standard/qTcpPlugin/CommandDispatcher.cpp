// CommandDispatcher.cpp
#include "CommandDispatcher.h"
#include "CcTcpServer.h"
#include <qjsondocument.h>
#include <FileIOFilter.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <cc2DViewportObject.h>
#include <QFileInfo>
#include <ccPolyline.h>
#include <ccMesh.h>
#include <ManualSegmentationTools.h>
#include <ccGLMatrix.h>
#include <ccHObjectCaster.h>
#include <ccShiftedObject.h>
#include <registrationTools.h>
#include "ccRegistrationTools.h"

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
	else if (action == "applyTransformation")
	{
		handleApplyTransformation(cmd["params"].toObject(), socket);
	}
	else if (action == "segment")
	{
		handleSegment(cmd["params"].toObject(), socket);
	}
	else if (action == "delete")
		handleDelete(cmd["params"].toObject(), socket);
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
	QString meshName     = params["meshName"].toString();
	QString binName      = params["binName"].toString();
	bool    keepInside   = params["keepInside"].toBool(true);
	bool    modifySource = params["modifySource"].toBool(false);
	QString outputName   = params["outputName"].toString();

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

	// 查找目标对象（Mesh 或 PointCloud）
	ccHObject* targetParent = findByName(dbRoot, meshName);
	if (!targetParent)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Object not found: " + meshName);
		return;
	}
	ccHObject* targetObj = targetParent->getChild(0);
	if (!targetObj)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Object has no children: " + meshName);
		return;
	}

	// 检查对象类型
	ccMesh*              targetMesh  = nullptr;
	ccGenericPointCloud* targetCloud = nullptr;

	if (targetObj->isKindOf(CC_TYPES::MESH))
	{
		targetMesh = ccHObjectCaster::ToMesh(targetObj);
	}
	else if (targetObj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		targetCloud = ccHObjectCaster::ToGenericPointCloud(targetObj);
	}
	else
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Object is not a mesh or point cloud: " + meshName);
		return;
	}

	if (!targetMesh && !targetCloud)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Failed to cast object to mesh or point cloud");
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
	glWindow->redraw();

	ccGLCameraParameters camera;
	glWindow->getGLCameraParameters(camera);

	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	// 将 polyline 顶点从 3D 转换到 2D 屏幕坐标
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
	ccGenericPointCloud* cloud = nullptr;
	if (targetMesh)
	{
		cloud = ccHObjectCaster::ToGenericPointCloud(targetMesh);
		if (!cloud)
		{
			delete segPoly2D;
			if (m_server)
				m_server->sendResponse(socket, false, "Mesh has no associated point cloud");
			return;
		}
	}
	else if (targetCloud)
	{
		cloud = targetCloud;
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

	// ---- 生成结果对象 ----
	if (targetMesh)
	{
		if (modifySource)
		{
			// createNewMeshFromSelection(true)：
			// - 返回值是"被选中的部分"（圆内）
			// - 同时从 targetMesh 中移除这些三角面（源被掏洞）
			ccMesh* removedPart = targetMesh->createNewMeshFromSelection(true);
			if (!removedPart || removedPart->size() == 0)
			{
				delete removedPart;
				delete segPoly2D;
				cloud->resetVisibilityArray();
				if (m_server)
					m_server->sendResponse(socket, false, "Segmentation result is empty");
				return;
			}

			// 圆内部分作为新 mesh 加入 DB
			QString newPartName = outputName.isEmpty() ? targetMesh->getName() + "_segmented" : outputName;
			removedPart->setName(newPartName);
			m_app->addToDB(removedPart); // ← 补上这一步

			// 源 mesh 已经被掏洞，刷新显示
			targetMesh->prepareDisplayForRefresh_recursive();
			cloud->resetVisibilityArray();
			delete segPoly2D;
			m_app->refreshAll();
			m_app->dispToConsole("[TcpPlugin] Source mesh punched, new part: " + newPartName);
			if (m_server)
				m_server->sendResponse(socket, true, "Source mesh punched, new part: " + newPartName);
		}
		else
		{
			// 生成新 mesh，源不动
			ccMesh* segmentedMesh = targetMesh->createNewMeshFromSelection(false);
			if (!segmentedMesh || segmentedMesh->size() == 0)
			{
				delete segmentedMesh;
				delete segPoly2D;
				cloud->resetVisibilityArray();
				if (m_server)
					m_server->sendResponse(socket, false, "Segmentation result is empty");
				return;
			}
			QString objectName = outputName.isEmpty() ? targetMesh->getName() + "_segmented" : outputName;
			segmentedMesh->setName(objectName);
			m_app->addToDB(segmentedMesh);
			cloud->resetVisibilityArray();
			delete segPoly2D;
			m_app->refreshAll();
			m_app->dispToConsole("[TcpPlugin] Segment OK: " + objectName);
			if (m_server)
				m_server->sendResponse(socket, true, "Segmentation completed: " + objectName);
		}
	}
	else if (targetCloud)
	{
		if (modifySource)
		{
			// visibilityArray 当前状态：
			// keepInside=true  → 圆内VISIBLE，圆外HIDDEN
			// keepInside=false → 圆外VISIBLE，圆内HIDDEN
			// createNewCloudFromVisibilitySelection(true)：
			// - 返回VISIBLE的点组成的新点云
			// - 同时从源点云中移除这些VISIBLE的点
			// 所以我们需要让"要删除的点"变成VISIBLE
			// 即：圆内的点应该是VISIBLE（keepInside=true时已经是这个状态，不需要反转）

			ccGenericPointCloud* removedPart = targetCloud->createNewCloudFromVisibilitySelection(true);
			if (!removedPart || removedPart->size() == 0)
			{
				delete removedPart;
				delete segPoly2D;
				cloud->resetVisibilityArray();
				if (m_server)
					m_server->sendResponse(socket, false, "Segmentation result is empty");
				return;
			}

			// 圆内被删除的点，作为新点云加入 DB
			QString newPartName = outputName.isEmpty() ? targetCloud->getName() + "_segmented" : outputName;
			removedPart->setName(newPartName);
			m_app->addToDB(removedPart);

			// 源点云已经被掏洞（圆内的点已移除），刷新显示
			targetCloud->prepareDisplayForRefresh_recursive();
			cloud->resetVisibilityArray();
			delete segPoly2D;
			m_app->refreshAll();
			m_app->dispToConsole("[TcpPlugin] Source cloud punched, removed part: " + newPartName);
			if (m_server)
				m_server->sendResponse(socket, true, "Source cloud punched, removed part: " + newPartName);
		}
		else
		{
			// 生成新点云，源不动
			ccGenericPointCloud* segmentedCloud = targetCloud->createNewCloudFromVisibilitySelection(false);
			if (!segmentedCloud || segmentedCloud->size() == 0)
			{
				delete segmentedCloud;
				delete segPoly2D;
				cloud->resetVisibilityArray();
				if (m_server)
					m_server->sendResponse(socket, false, "Segmentation result is empty");
				return;
			}
			QString objectName = outputName.isEmpty() ? targetCloud->getName() + "_segmented" : outputName;
			segmentedCloud->setName(objectName);
			m_app->addToDB(segmentedCloud);
			cloud->resetVisibilityArray();
			delete segPoly2D;
			m_app->refreshAll();
			m_app->dispToConsole("[TcpPlugin] Segment OK: " + objectName);
			if (m_server)
				m_server->sendResponse(socket, true, "Segmentation completed: " + objectName);
		}
	}
}



void CommandDispatcher::handleApplyTransformation(const QJsonObject& params, QTcpSocket* socket)
{
	// ---- 1. 解析参数 ----
	QString objectName    = params["name"].toString();
	QString matrixText    = params["matrix"].toString();
	bool    applyToGlobal = params["applyToGlobal"].toBool(false);
	bool    inverse       = params["inverse"].toBool(false);

	if (objectName.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing 'name' parameter");
		return;
	}
	if (matrixText.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing 'matrix' parameter");
		return;
	}

	// ---- 2. 解析矩阵 ----
	bool        valid = false;
	ccGLMatrixd mat   = ccGLMatrixd::FromString(matrixText, valid);
	if (!valid)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Invalid matrix format");
		return;
	}
	if (inverse)
		mat.invert();

	// ---- 3. 查找对象（复用你的 findByName 逻辑）----
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

	ccHObject* entity = findByName(dbRoot, objectName);
	if (!entity)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Object not found: " + objectName);
		return;
	}

	// ---- 4. 计算实际应用的 transMat（对齐 CC 源码逻辑）----
	ccGLMatrixd transMat = mat;

	if (applyToGlobal)
	{
		ccShiftedObject* shiftedEntity = dynamic_cast<ccShiftedObject*>(entity);
		if (shiftedEntity)
		{
			CCVector3d globalShift = shiftedEntity->getGlobalShift();
			double     globalScale = shiftedEntity->getGlobalScale();

			CCVector3d rotatedGlobalShift = globalShift;
			mat.applyRotation(rotatedGlobalShift);
			CCVector3d localTranslation = globalScale * (globalShift - rotatedGlobalShift + mat.getTranslationAsVec3D());

			transMat.setTranslation(localTranslation);
		}
	}

	// ---- 5. 锁定检查（点云类型才需要）----
	bool                 lockedVertices = false;
	ccGenericPointCloud* cloud          = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
	if (cloud && lockedVertices)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Vertices are locked: " + objectName);
		return;
	}
	// 变换前先删除 octree，避免依赖回调问题
	if (cloud)
		cloud->deleteOctree();

	// ---- 6. 应用变换（对齐 CC 源码）----
	entity->setGLTransformation(ccGLMatrix(transMat.data()));
	entity->applyGLTransformation_recursive();
	entity->prepareDisplayForRefresh_recursive();

	m_app->updateUI();
	m_app->refreshAll();

	m_app->dispToConsole("[TcpPlugin] Transformation applied to: " + objectName);
	if (m_server)
		m_server->sendResponse(socket, true, "Transformation applied to: " + objectName);
}


void CommandDispatcher::handleDelete(const QJsonObject& params, QTcpSocket* socket)
{
	QString objectName = params["name"].toString();
	if (objectName.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing 'name' parameter");
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

	ccHObject* target = findByName(dbRoot, objectName);
	if (!target)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Object not found: " + objectName);
		return;
	}

	// removeFromDB 会连同子节点一起删除
	m_app->removeFromDB(target);
	m_app->refreshAll();

	m_app->dispToConsole("[TcpPlugin] Deleted: " + objectName);
	if (m_server)
		m_server->sendResponse(socket, true, "Deleted: " + objectName);
}


void CommandDispatcher::handleICP(const QJsonObject& params, QTcpSocket* socket)
{
	// ---- 1. 解析参数 ----
	QString dataName  = params["data"].toString();  // 要移动的点云
	QString modelName = params["model"].toString(); // 参考点云（不动）

	if (dataName.isEmpty() || modelName.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing 'data' or 'model' parameter");
		return;
	}

	// ICP 参数，带默认值
	double minRMSDecrease       = params["minRMSDecrease"].toDouble(1e-5);
	int    maxIterations        = params["maxIterations"].toInt(20);
	double finalOverlapRatio    = params["finalOverlap"].toDouble(1.0); // 0~1，对应 0%~100%
	bool   adjustScale          = params["adjustScale"].toBool(false);
	bool   removeFarthestPoints = params["removeFarthestPoints"].toBool(false);
	int    samplingLimit        = params["samplingLimit"].toInt(50000);
	int    maxThreadCount       = params["maxThreadCount"].toInt(0); // 0 = 自动

	// ---- 2. 查找对象 ----
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

	// 查找 data 对象（点云或 mesh）
	ccHObject* dataParent = findByName(dbRoot, dataName)->getChild(0);
	if (!dataParent)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Data object not found: " + dataName);
		return;
	}
	ccHObject* dataObj = nullptr;
	for (unsigned i = 0; i < dataParent->getChildrenNumber(); ++i)
	{
		ccHObject* child = dataParent->getChild(i);
		if (child->isKindOf(CC_TYPES::POINT_CLOUD) || child->isKindOf(CC_TYPES::MESH))
		{
			dataObj = child;
			break;
		}
	}
	if (!dataObj)
		dataObj = dataParent; // 直接就是点云/mesh 本身

	// 查找 model 对象
	ccHObject* modelParent = findByName(dbRoot, modelName)->getChild(0);
	if (!modelParent)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Model object not found: " + modelName);
		return;
	}
	ccHObject* modelObj = nullptr;
	for (unsigned i = 0; i < modelParent->getChildrenNumber(); ++i)
	{
		ccHObject* child = modelParent->getChild(i);
		if (child->isKindOf(CC_TYPES::POINT_CLOUD) || child->isKindOf(CC_TYPES::MESH))
		{
			modelObj = child;
			break;
		}
	}
	if (!modelObj)
		modelObj = modelParent;

	// 类型检查
	if ((!dataObj->isKindOf(CC_TYPES::POINT_CLOUD) && !dataObj->isKindOf(CC_TYPES::MESH))
	    || (!modelObj->isKindOf(CC_TYPES::POINT_CLOUD) && !modelObj->isKindOf(CC_TYPES::MESH)))
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Both objects must be point clouds or meshes");
		return;
	}

	// ---- 3. 构建 ICP 参数 ----
	CCCoreLib::ICPRegistrationTools::Parameters parameters;
	parameters.convType                 = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE;
	parameters.minRMSDecrease           = minRMSDecrease;
	parameters.nbMaxIterations          = static_cast<unsigned>(maxIterations);
	parameters.adjustScale              = adjustScale;
	parameters.filterOutFarthestPoints  = removeFarthestPoints;
	parameters.samplingLimit            = static_cast<unsigned>(samplingLimit);
	parameters.finalOverlapRatio        = finalOverlapRatio;
	parameters.transformationFilters    = CCCoreLib::RegistrationTools::SKIP_NONE;
	parameters.maxThreadCount           = maxThreadCount;
	parameters.useC2MSignedDistances    = false;
	parameters.robustC2MSignedDistances = false;
	parameters.normalsMatching          = CCCoreLib::ICPRegistrationTools::NO_NORMAL;

	// ---- 4. 执行 ICP ----
	ccGLMatrix transMat;
	double     finalError      = 0.0;
	double     finalScale      = 1.0;
	unsigned   finalPointCount = 0;

	if ((!dataObj->isKindOf(CC_TYPES::POINT_CLOUD) && !dataObj->isKindOf(CC_TYPES::MESH))
	    || (!modelObj->isKindOf(CC_TYPES::POINT_CLOUD) && !modelObj->isKindOf(CC_TYPES::MESH)))
	{
		//ccConsole::Error(tr("Select 2 point clouds or meshes!"));
		return;
	}


	bool success = ccRegistrationTools::ICP(
	    dataObj,
	    modelObj,
	    transMat,
	    finalScale,
	    finalError,
	    finalPointCount,
	    parameters,
	    false, // useDataSFAsWeights
	    false, // useModelSFAsWeights
	   (QWidget*)(m_app->getMainWindow()));

	if (!success)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "ICP failed");
		return;
	}

	// ---- 5. 应用变换 ----
	ccGenericPointCloud* pc = nullptr;

	if (dataObj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		pc = ccHObjectCaster::ToGenericPointCloud(dataObj);
	}
	else if (dataObj->isKindOf(CC_TYPES::MESH))
	{
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(dataObj);
		pc                  = mesh->getAssociatedCloud();
		if (pc && pc->isLocked())
		{
			if (m_server)
				m_server->sendResponse(socket, false, "Mesh vertices are locked, cannot apply transformation");
			return;
		}
	}

	if (!pc)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Failed to get point cloud from data object");
		return;
	}

	// 应用变换（对齐 CC 源码）
	bool modelIsChildOfData = dataObj->isAncestorOf(modelObj);
	if (modelIsChildOfData)
	{
		pc->applyRigidTransformation(transMat);
	}
	else
	{
		pc->applyGLTransformation_recursive(&transMat);
	}

	// mesh 需要刷新包围盒
	if (dataObj->isKindOf(CC_TYPES::MESH))
	{
		ccHObjectCaster::ToGenericMesh(dataObj)->refreshBB();
	}

	// 同步 Global Shift（对齐 CC 源码逻辑）
	ccGenericPointCloud* refPC = ccHObjectCaster::ToGenericPointCloud(modelObj);
	if (refPC && refPC->isShifted())
	{
		const CCVector3d& Pshift = refPC->getGlobalShift();
		double            scale  = refPC->getGlobalScale();
		pc->setGlobalShift(Pshift);
		pc->setGlobalScale(scale);
		m_app->dispToConsole(
		    QString("[TcpPlugin][ICP] Global shift updated: (%1,%2,%3) x%4")
		        .arg(Pshift.x)
		        .arg(Pshift.y)
		        .arg(Pshift.z)
		        .arg(scale));
	}

	dataObj->prepareDisplayForRefresh_recursive();
	m_app->refreshAll();
	m_app->updateUI();

	// ---- 6. 返回结果 ----
	QString matrixStr = transMat.toString(6, ' ');
	m_app->dispToConsole(QString("[TcpPlugin][ICP] Final RMS: %1 (on %2 points)").arg(finalError).arg(finalPointCount));
	m_app->dispToConsole(QString("[TcpPlugin][ICP] Transformation matrix:\n") + matrixStr);

	// 构建响应 JSON，包含 RMS 和变换矩阵
	QJsonObject result;
	result["finalRMS"]        = finalError;
	result["finalPointCount"] = static_cast<int>(finalPointCount);
	result["finalScale"]      = finalScale;
	result["matrix"]          = matrixStr;

	QJsonDocument doc(result);
	if (m_server)
		m_server->sendResponse(socket, true, doc.toJson(QJsonDocument::Compact));
}
