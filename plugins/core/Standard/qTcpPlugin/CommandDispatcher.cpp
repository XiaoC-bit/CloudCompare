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
#include <GeometricalAnalysisTools.h>
#include <ccSphere.h>
#include "ccRegistrationTools.h"

CommandDispatcher::CommandDispatcher(ccMainAppInterface* app, CcTcpServer* server, QObject* parent)
    : QObject(parent)
    , m_app(app)
    , m_server(server)
{}

// ============================================================
//  Helpers
// ============================================================

void CommandDispatcher::sendOk(QTcpSocket* socket, const QString& msg)
{
	if (m_server)
		m_server->sendResponse(socket, true, msg);
}

void CommandDispatcher::sendError(QTcpSocket* socket, const QString& msg)
{
	if (m_server)
		m_server->sendResponse(socket, false, msg);
}

ccHObject* CommandDispatcher::getDbRoot(QTcpSocket* socket)
{
	ccHObject* root = m_app->dbRootObject();
	if (!root)
		sendError(socket, "DB root is null");
	return root;
}

ccHObject* CommandDispatcher::findByName(ccHObject* node, const QString& name)
{
	if (node->getName() == name)
		return node;
	for (unsigned i = 0; i < node->getChildrenNumber(); ++i)
		if (ccHObject* found = findByName(node->getChild(i), name))
			return found;
	return nullptr;
}

// ============================================================
//  Dispatch
// ============================================================

void CommandDispatcher::dispatch(QJsonObject cmd, QTcpSocket* socket)
{
	const QString     action = cmd["action"].toString();
	const QJsonObject params = cmd["params"].toObject();

	if      (action == "load")                handleLoad(params, socket);
	else if (action == "filter")              handleFilter(params, socket);
	else if (action == "icp")                 handleICP(params, socket);
	else if (action == "camera")              handleCamera(params, socket);
	else if (action == "applyViewport")       handleApplyViewport(params, socket);
	else if (action == "applyTransformation") handleApplyTransformation(params, socket);
	else if (action == "segment")             handleSegment(params, socket);
	else if (action == "delete")              handleDelete(params, socket);
	else if (action == "fit")                 handleFit(params, socket);
	else if (action == "clearDB")             handleClearDB(params, socket);
	else
	{
		m_app->dispToConsole("[TcpPlugin] Unknown action: " + action, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		sendError(socket, "Unknown action: " + action);
	}
}

// ============================================================
//  Handlers
// ============================================================

void CommandDispatcher::handleLoad(const QJsonObject& params, QTcpSocket* socket)
{
	const QString path = params["path"].toString();
	if (path.isEmpty())
	{
		sendError(socket, "Empty path");
		return;
	}

	FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(QFileInfo(path).suffix());
	if (!filter)
	{
		m_app->dispToConsole("[TcpPlugin] Unsupported file format", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		sendError(socket, "Unsupported file format");
		return;
	}

	ccHObject* container = new ccHObject();
	if (filter->loadFile(path, *container, FileIOFilter::LoadParameters()) == CC_FERR_NO_ERROR)
	{
		const QString modelName = params["name"].toString();
		if (!modelName.isEmpty())
			container->setName(modelName);

		m_app->addToDB(container);
		m_app->dispToConsole("[TcpPlugin] Loaded: " + path);
		sendOk(socket, "Loaded: " + path);
	}
	else
	{
		m_app->dispToConsole("[TcpPlugin] Failed to load: " + path, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		sendError(socket, "Failed to load: " + path);
		delete container;
	}
}

void CommandDispatcher::handleCamera(const QJsonObject& params, QTcpSocket* socket)
{
	auto* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		sendError(socket, "No active GL window");
		return;
	}

	if (!params.contains("viewPreset"))
	{
		sendError(socket, "Missing viewPreset parameter");
		return;
	}

	const QString preset = params["viewPreset"].toString();
	if      (preset == "top")   glWindow->setView(CC_TOP_VIEW);
	else if (preset == "front") glWindow->setView(CC_FRONT_VIEW);
	else if (preset == "iso")   glWindow->setView(CC_ISO_VIEW_1);

	glWindow->redraw();
	sendOk(socket, "Camera view set to: " + preset);
}

void CommandDispatcher::handleFilter(const QJsonObject& params, QTcpSocket* socket)
{
	sendError(socket, "Filter not implemented");
}

void CommandDispatcher::handleApplyViewport(const QJsonObject& params, QTcpSocket* socket)
{
	const QString name = params["name"].toString();
	if (name.isEmpty())
	{
		sendError(socket, "Empty name parameter");
		return;
	}

	ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		sendError(socket, "No active GL window");
		return;
	}

	// Search only top-level children (intentional: name refers to a root-level group)
	ccHObject* rootObject = nullptr;
	ccHObject* dbRoot     = m_app->dbRootObject();
	if (dbRoot)
	{
		for (unsigned i = 0; i < dbRoot->getChildrenNumber(); ++i)
		{
			ccHObject* child = dbRoot->getChild(i);
			if (child && child->getName() == name)
			{
				rootObject = child->getChild(0);
				break;
			}
		}
	}

	if (!rootObject)
	{
		m_app->dispToConsole("[TcpPlugin] Object not found: " + name, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		sendError(socket, "Object not found: " + name);
		return;
	}

	cc2DViewportObject* viewportObject = nullptr;
	for (unsigned i = 0; i < rootObject->getChildrenNumber(); ++i)
	{
		ccHObject* obj = rootObject->getChild(i);
		if (obj && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT))
		{
			viewportObject = static_cast<cc2DViewportObject*>(obj);
			break;
		}
	}

	if (!viewportObject)
	{
		m_app->dispToConsole("[TcpPlugin] Viewport object not found in hierarchy", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		sendError(socket, "Viewport object not found in hierarchy");
		return;
	}

	cc2DViewportObject* viewport = ccHObjectCaster::To2DViewportObject(viewportObject);
	assert(viewport);
	if (!viewport)
		return;

	glWindow->setViewportParameters(viewport->getParameters());
	glWindow->redraw();
}

void CommandDispatcher::handleSegment(const QJsonObject& params, QTcpSocket* socket)
{
	const QString meshName     = params["meshName"].toString();
	const QString binName      = params["binName"].toString();
	const bool    keepInside   = params["keepInside"].toBool(true);
	const bool    modifySource = params["modifySource"].toBool(false);
	const QString outputName   = params["outputName"].toString();

	if (meshName.isEmpty() || binName.isEmpty())
	{
		sendError(socket, "Missing meshName or binName");
		return;
	}

	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	// Find target object (mesh or point cloud)
	ccHObject* targetParent = findByName(root, meshName);
	if (!targetParent)
	{
		sendError(socket, "Object not found: " + meshName);
		return;
	}
	ccHObject* targetObj = targetParent->getChild(0);
	if (!targetObj)
	{
		sendError(socket, "Object has no children: " + meshName);
		return;
	}

	ccMesh*              targetMesh  = nullptr;
	ccGenericPointCloud* targetCloud = nullptr;
	if (targetObj->isKindOf(CC_TYPES::MESH))
		targetMesh = ccHObjectCaster::ToMesh(targetObj);
	else if (targetObj->isKindOf(CC_TYPES::POINT_CLOUD))
		targetCloud = ccHObjectCaster::ToGenericPointCloud(targetObj);
	else
	{
		sendError(socket, "Object is not a mesh or point cloud: " + meshName);
		return;
	}

	// Find bin and extract polyline + viewport from its hierarchy
	ccHObject* binRoot = findByName(root, binName);
	if (!binRoot)
	{
		sendError(socket, "Bin root not found: " + binName);
		return;
	}

	ccPolyline*         segPoly  = nullptr;
	cc2DViewportObject* vpObject = nullptr;
	std::function<void(ccHObject*)> findInHierarchy = [&](ccHObject* obj)
	{
		if (!segPoly  && obj->isKindOf(CC_TYPES::POLY_LINE))
			segPoly = static_cast<ccPolyline*>(obj);
		if (!vpObject && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT))
			vpObject = static_cast<cc2DViewportObject*>(obj);
		for (unsigned i = 0; i < obj->getChildrenNumber(); ++i)
			findInHierarchy(obj->getChild(i));
	};
	findInHierarchy(binRoot);

	if (!segPoly)   { sendError(socket, "Polyline not found in bin");        return; }
	if (!segPoly->isClosed()) { sendError(socket, "Polyline is not closed"); return; }
	if (!vpObject)  { sendError(socket, "Viewport object not found in bin"); return; }

	// Apply viewport and capture camera
	ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		sendError(socket, "No active GL window");
		return;
	}
	glWindow->setViewportParameters(vpObject->getParameters());
	glWindow->redraw();

	ccGLCameraParameters camera;
	glWindow->getGLCameraParameters(camera);

	const double half_w = camera.viewport[2] / 2.0;
	const double half_h = camera.viewport[3] / 2.0;

	// Project polyline vertices to 2D screen coordinates
	ccPointCloud* polyVertices2D = new ccPointCloud();
	ccPolyline*   segPoly2D      = new ccPolyline(polyVertices2D);
	segPoly2D->addChild(polyVertices2D);

	CCCoreLib::GenericIndexedCloudPersist* vertices = segPoly->getAssociatedCloud();
	const bool mode3D = !segPoly->is2DMode();

	if (!polyVertices2D->reserve(vertices->size()) || !segPoly2D->reserve(segPoly->size()))
	{
		delete segPoly2D;
		sendError(socket, "Not enough memory for polyline conversion");
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
		segPoly2D->addPointIndex(segPoly->getPointGlobalIndex(j));
	segPoly2D->setClosed(segPoly->isClosed());

	// Check if polygon is fully inside viewport
	bool polyInsideViewport = true;
	for (unsigned i = 0; i < segPoly2D->size(); ++i)
	{
		const CCVector3* P2D = segPoly2D->getPoint(i);
		if (P2D->x < -half_w || P2D->x > half_w || P2D->y < -half_h || P2D->y > half_h)
		{
			polyInsideViewport = false;
			break;
		}
	}

	// Resolve the point cloud to classify
	ccGenericPointCloud* cloud = targetMesh ? ccHObjectCaster::ToGenericPointCloud(targetMesh) : targetCloud;
	if (targetMesh && !cloud)
	{
		delete segPoly2D;
		sendError(socket, "Mesh has no associated point cloud");
		return;
	}

	if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray())
	{
		delete segPoly2D;
		sendError(socket, "Failed to initialize visibility array");
		return;
	}

	// Project and classify each point against the polygon
	ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
	for (int i = 0; i < static_cast<int>(cloud->size()); ++i)
	{
		if (visibilityArray[i] != CCCoreLib::POINT_VISIBLE)
			continue;

		CCVector3d Q2D;
		bool       pointInFrustum = false;
		camera.project(*cloud->getPoint(i), Q2D, &pointInFrustum);

		bool pointInside = false;
		if (pointInFrustum || !polyInsideViewport)
		{
			CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
			              static_cast<PointCoordinateType>(Q2D.y - half_h));
			pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, segPoly2D);
		}

		visibilityArray[i] = (keepInside != pointInside) ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE;
	}

	// Generate result object
	auto finishWithEmpty = [&]()
	{
		delete segPoly2D;
		cloud->resetVisibilityArray();
		sendError(socket, "Segmentation result is empty");
	};

	if (targetMesh)
	{
		ccMesh* result = targetMesh->createNewMeshFromSelection(modifySource);
		if (!result || result->size() == 0) { delete result; finishWithEmpty(); return; }

		const QString resultName = outputName.isEmpty() ? targetMesh->getName() + "_segmented" : outputName;
		result->setName(resultName);
		m_app->addToDB(result);
		if (modifySource)
			targetMesh->prepareDisplayForRefresh_recursive();

		cloud->resetVisibilityArray();
		delete segPoly2D;
		m_app->refreshAll();

		const QString msg = modifySource ? QString ("Source mesh punched, new part: ") + resultName
		                                 : QString ("Segmentation completed: ") + resultName;
		m_app->dispToConsole("[TcpPlugin] " + msg);
		sendOk(socket, msg);
	}
	else
	{
		ccGenericPointCloud* result = targetCloud->createNewCloudFromVisibilitySelection(modifySource);
		if (!result || result->size() == 0) { delete result; finishWithEmpty(); return; }

		const QString resultName = outputName.isEmpty() ? targetCloud->getName() + "_segmented" : outputName;
		result->setName(resultName);
		m_app->addToDB(result);
		if (modifySource)
			targetCloud->prepareDisplayForRefresh_recursive();

		cloud->resetVisibilityArray();
		delete segPoly2D;
		m_app->refreshAll();

		const QString msg = modifySource ? QString("Source cloud punched, removed part: ") + resultName
		                                 : QString ("Segmentation completed: " )+ resultName;
		m_app->dispToConsole("[TcpPlugin] " + msg);
		sendOk(socket, msg);
	}
}

void CommandDispatcher::handleApplyTransformation(const QJsonObject& params, QTcpSocket* socket)
{
	const QString objectName    = params["name"].toString();
	const QString matrixText    = params["matrix"].toString();
	const bool    applyToGlobal = params["applyToGlobal"].toBool(false);
	const bool    inverse       = params["inverse"].toBool(false);

	if (objectName.isEmpty()) { sendError(socket, "Missing 'name' parameter");   return; }
	if (matrixText.isEmpty()) { sendError(socket, "Missing 'matrix' parameter"); return; }

	bool valid = false;
	ccGLMatrixd mat = ccGLMatrixd::FromString(matrixText, valid);
	if (!valid)
	{
		sendError(socket, "Invalid matrix format");
		return;
	}
	if (inverse)
		mat.invert();

	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	ccHObject* entity = findByName(root, objectName);
	if (!entity)
	{
		sendError(socket, "Object not found: " + objectName);
		return;
	}

	// Compute the actual local transformation matrix (aligns with CC source logic)
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

	// Check for locked vertices
	bool                 lockedVertices = false;
	ccGenericPointCloud* cloud          = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
	if (cloud && lockedVertices)
	{
		sendError(socket, "Vertices are locked: " + objectName);
		return;
	}
	if (cloud)
		cloud->deleteOctree();

	entity->setGLTransformation(ccGLMatrix(transMat.data()));
	entity->applyGLTransformation_recursive();
	entity->prepareDisplayForRefresh_recursive();

	m_app->updateUI();
	m_app->refreshAll();

	m_app->dispToConsole("[TcpPlugin] Transformation applied to: " + objectName);
	sendOk(socket, "Transformation applied to: " + objectName);
}

void CommandDispatcher::handleDelete(const QJsonObject& params, QTcpSocket* socket)
{
	const QString objectName = params["name"].toString();
	if (objectName.isEmpty())
	{
		sendError(socket, "Missing 'name' parameter");
		return;
	}

	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	ccHObject* target = findByName(root, objectName);
	if (!target)
	{
		sendError(socket, "Object not found: " + objectName);
		return;
	}

	m_app->removeFromDB(target);
	m_app->refreshAll();

	m_app->dispToConsole("[TcpPlugin] Deleted: " + objectName);
	sendOk(socket, "Deleted: " + objectName);
}

void CommandDispatcher::handleICP(const QJsonObject& params, QTcpSocket* socket)
{
	const QString dataName  = params["data"].toString();
	const QString modelName = params["model"].toString();

	if (dataName.isEmpty() || modelName.isEmpty())
	{
		sendError(socket, "Missing 'data' or 'model' parameter");
		return;
	}

	// Build ICP parameters with defaults
	CCCoreLib::ICPRegistrationTools::Parameters icpParams;
	icpParams.convType                  = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE;
	icpParams.minRMSDecrease            = params["minRMSDecrease"].toDouble(1e-5);
	icpParams.nbMaxIterations           = static_cast<unsigned>(params["maxIterations"].toInt(20));
	icpParams.adjustScale               = params["adjustScale"].toBool(false);
	icpParams.filterOutFarthestPoints   = params["removeFarthestPoints"].toBool(false); // experimental, may crash
	icpParams.samplingLimit             = static_cast<unsigned>(params["samplingLimit"].toInt(50000));
	icpParams.finalOverlapRatio         = params["finalOverlap"].toDouble(1.0);
	icpParams.transformationFilters     = CCCoreLib::RegistrationTools::SKIP_NONE;
	icpParams.maxThreadCount            = params["maxThreadCount"].toInt(0); // 0 = auto
	icpParams.useC2MSignedDistances     = false;
	icpParams.robustC2MSignedDistances  = true;
	icpParams.normalsMatching           = CCCoreLib::ICPRegistrationTools::NO_NORMAL;

	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	// Find a point cloud or mesh by name, searching parent first then children
	auto findCloudOrMesh = [&](const QString& name, const QString& role) -> ccHObject*
	{
		ccHObject* parent = findByName(root, name);
		if (!parent)
		{
			sendError(socket, role + " object not found: " + name);
			return nullptr;
		}
		for (unsigned i = 0; i < parent->getChildrenNumber(); ++i)
		{
			ccHObject* child = parent->getChild(i);
			if (child->isKindOf(CC_TYPES::POINT_CLOUD) || child->isKindOf(CC_TYPES::MESH))
				return child;
		}
		return parent;
	};

	ccHObject* dataObj  = findCloudOrMesh(dataName,  "Data");
	if (!dataObj)  return;
	ccHObject* modelObj = findCloudOrMesh(modelName, "Model");
	if (!modelObj) return;

	if ((!dataObj->isKindOf(CC_TYPES::POINT_CLOUD) && !dataObj->isKindOf(CC_TYPES::MESH))
	    || (!modelObj->isKindOf(CC_TYPES::POINT_CLOUD) && !modelObj->isKindOf(CC_TYPES::MESH)))
	{
		sendError(socket, "Both objects must be point clouds or meshes");
		return;
	}

	// Run ICP
	ccGLMatrix transMat;
	double     finalError      = 0.0;
	double     finalScale      = 1.0;
	unsigned   finalPointCount = 0;

	bool success = ccRegistrationTools::ICP(
	    dataObj, modelObj, transMat, finalScale, finalError, finalPointCount,
	    icpParams, false, false, (QWidget*)(m_app->getMainWindow()));

	if (!success)
	{
		sendError(socket, "ICP failed");
		return;
	}

	// Retrieve point cloud to transform
	ccGenericPointCloud* pc = nullptr;
	if (dataObj->isKindOf(CC_TYPES::POINT_CLOUD))
	{
		pc = ccHObjectCaster::ToGenericPointCloud(dataObj);
	}
	else if (dataObj->isKindOf(CC_TYPES::MESH))
	{
		ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(dataObj);
		pc = mesh->getAssociatedCloud();
		if (pc && pc->isLocked())
		{
			sendError(socket, "Mesh vertices are locked, cannot apply transformation");
			return;
		}
	}

	if (!pc)
	{
		sendError(socket, "Failed to get point cloud from data object");
		return;
	}

	if (dataObj->isAncestorOf(modelObj))
		pc->applyRigidTransformation(transMat);
	else
		pc->applyGLTransformation_recursive(&transMat);

	if (dataObj->isKindOf(CC_TYPES::MESH))
		ccHObjectCaster::ToGenericMesh(dataObj)->refreshBB();

	// Sync global shift from model (aligns with CC source logic)
	ccGenericPointCloud* refPC = ccHObjectCaster::ToGenericPointCloud(modelObj);
	if (refPC && refPC->isShifted())
	{
		const CCVector3d& Pshift = refPC->getGlobalShift();
		const double      scale  = refPC->getGlobalScale();
		pc->setGlobalShift(Pshift);
		pc->setGlobalScale(scale);
		m_app->dispToConsole(
		    QString("[TcpPlugin][ICP] Global shift updated: (%1,%2,%3) x%4")
		        .arg(Pshift.x).arg(Pshift.y).arg(Pshift.z).arg(scale));
	}

	dataObj->prepareDisplayForRefresh_recursive();
	m_app->refreshAll();
	m_app->updateUI();

	const QString matrixStr = transMat.toString(6, ' ');
	m_app->dispToConsole(QString("[TcpPlugin][ICP] Final RMS: %1 (on %2 points)").arg(finalError).arg(finalPointCount));
	m_app->dispToConsole(QString("[TcpPlugin][ICP] Transformation matrix:\n") + matrixStr);

	QJsonObject result;
	result["finalRMS"]        = finalError;
	result["finalPointCount"] = static_cast<int>(finalPointCount);
	result["finalScale"]      = finalScale;
	result["matrix"]          = matrixStr;
	sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact));
}

void CommandDispatcher::handleFit(const QJsonObject& params, QTcpSocket* socket)
{
	const QString type = params["type"].toString();
	if (type == "sphere")
		handleFitSphere(params, socket);
	else
		sendError(socket, "Unknown fit type: " + type);
}

void CommandDispatcher::handleFitSphere(const QJsonObject& params, QTcpSocket* socket)
{
	const QString objectName       = params["name"].toString();
	const double  outliersRatio    = params["outliersRatio"].toDouble(0.5);
	const double  confidence       = params["confidence"].toDouble(0.99);
	const bool    autoDetectRadius = params["autoDetectRadius"].toBool(true);
	const double  radius           = params["radius"].toDouble(0.0);

	if (objectName.isEmpty())
	{
		sendError(socket, "Missing 'name' parameter");
		return;
	}

	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	// Find a point cloud by name (direct match or first child)
	auto findPointCloud = [&](const QString& name) -> ccPointCloud*
	{
		ccHObject* found = findByName(root, name);
		if (!found)
			return nullptr;
		if (found->isA(CC_TYPES::POINT_CLOUD))
			return static_cast<ccPointCloud*>(found);
		for (unsigned i = 0; i < found->getChildrenNumber(); ++i)
			if (found->getChild(i)->isA(CC_TYPES::POINT_CLOUD))
				return static_cast<ccPointCloud*>(found->getChild(i));
		return nullptr;
	};

	ccPointCloud* cloud = findPointCloud(objectName);
	if (!cloud)
	{
		sendError(socket, "Point cloud not found: " + objectName);
		return;
	}

	// Run sphere fitting
	CCVector3           center;
	PointCoordinateType fitRadius = autoDetectRadius ? 0 : static_cast<PointCoordinateType>(radius);
	double              rms       = std::numeric_limits<double>::quiet_NaN();

	auto fitResult = CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(
	    cloud, outliersRatio, center, fitRadius, rms, !autoDetectRadius, nullptr, confidence);

	if (fitResult != CCCoreLib::GeometricalAnalysisTools::NoError)
	{
		sendError(socket, QString("Sphere fitting failed on '%1' (error code: %2)").arg(objectName).arg(fitResult));
		return;
	}

	m_app->dispToConsole(
	    QString("[TcpPlugin][FitSphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6]")
	        .arg(cloud->getName()).arg(center.x).arg(center.y).arg(center.z).arg(fitRadius).arg(rms));

	// Create sphere object and add to DB (aligns with CC source)
	ccGLMatrix trans;
	trans.setTranslation(center);
	ccSphere* sphere = new ccSphere(fitRadius, &trans, QString("Sphere r=%1").arg(fitRadius));
	sphere->copyGlobalShiftAndScale(*cloud);
	sphere->setMetaData("RMS", rms);
	cloud->addChild(sphere);
	sphere->prepareDisplayForRefresh();
	m_app->addToDB(sphere, false, false, false);

	m_app->refreshAll();
	m_app->updateUI();

	QJsonObject resultJson;
	resultJson["center_x"] = static_cast<double>(center.x);
	resultJson["center_y"] = static_cast<double>(center.y);
	resultJson["center_z"] = static_cast<double>(center.z);
	resultJson["radius"]   = static_cast<double>(fitRadius);
	resultJson["rms"]      = rms;
	sendOk(socket, QJsonDocument(resultJson).toJson(QJsonDocument::Compact));
}

void CommandDispatcher::handleClearDB(const QJsonObject& params, QTcpSocket* socket)
{
	ccHObject* root = getDbRoot(socket);
	if (!root)
		return;

	// Collect first, then delete (avoid modifying while iterating)
	std::vector<ccHObject*> toDelete;
	toDelete.reserve(root->getChildrenNumber());
	for (unsigned i = 0; i < root->getChildrenNumber(); ++i)
		toDelete.push_back(root->getChild(i));

	for (ccHObject* obj : toDelete)
		m_app->removeFromDB(obj);

	m_app->refreshAll();
	m_app->updateUI();

	m_app->dispToConsole("[TcpPlugin] DB cleared");
	sendOk(socket, "DB cleared");
}
