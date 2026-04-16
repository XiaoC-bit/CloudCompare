#include "PointCloudService.h"

#include <CloudSamplingTools.h>
#include <FileIOFilter.h>
#include <GeometricalAnalysisTools.h>
#include <ManualSegmentationTools.h>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QMetaObject>
#include <QTextStream>
#include <QThread>
#include <QUuid>
#include <cc2DViewportObject.h>
#include <ccGLMatrix.h>
#include <ccGLWindowInterface.h>
#include <ccHObject.h>
#include <ccHObjectCaster.h>
#include <ccMainAppInterface.h>
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccShiftedObject.h>
#include <ccSphere.h>
#include <fstream>
#include <limits>
#include <qapplication.h>
#include <qjsonarray.h>
#include <qjsondocument.h>
#include <registrationTools.h>
#include <stdexcept>

#include "CalibrationDialog.h"
#include "CommLogger.h"
#include "LJS8_IF.h"
#include "LJS8_ErrorCode.h"
#include "LJS8_ACQ.h"
#include "ccRegistrationTools.h"

#ifndef CC_ORIGINAL_CLOUD_INDEX_SF_NAME
#define CC_ORIGINAL_CLOUD_INDEX_SF_NAME "Original cloud index"
#endif

static int    COLLECT_VALUE = 32768;
static double INVALID_VALUE = -999.9999;

namespace
{
	const QString MACHINE_HOST                   = "localhost";
	const quint16 MACHINE_PORT                   = 20002;
	const QString MACHINE_DEVICE_NAME            = "CNC_1";
	const QString MACHINE_DEVICE_TYPE            = "Cnc";
	const QString CALIBRATION_CNC_FILE           = "O1236";
	const QString CALIBRATION_CNC_PATH           = "/c/";
	const double  CALIBRATION_RADIUS             = 12.5;
	const double  CALIBRATION_RMS_THRESHOLD      = 0.012;
	const double  CALIBRATION_RESIDUAL_THRESHOLD = 0.12;
	const int     CALIBRATION_MAX_FIT_RETRIES    = 3;
} // namespace

PointCloudService::PointCloudService(ccMainAppInterface* app, QObject* parent)
    : QObject(parent)
    , m_app(app)
    , m_machineSocket(nullptr)
    , m_workerMachineSocket(nullptr)
    , m_Status(MachineStatus::Idle)
{
	// 设置状态文件路径
	QString appDir      = QCoreApplication::applicationDirPath();
	m_statusFilePath    = appDir + "/Template/calibration_status.json";
	m_Status = MachineStatus::Idle;
	// 加载之前的标定结果
	QFile statusFile(m_statusFilePath);
	if (statusFile.exists() && statusFile.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QByteArray      data = statusFile.readAll();
		QJsonParseError parseError;
		QJsonDocument   doc = QJsonDocument::fromJson(data, &parseError);
		if (parseError.error == QJsonParseError::NoError && doc.isObject())
		{
			m_calibrationResult = doc.object();
			if (m_calibrationResult.contains("calibrationResult") && m_calibrationResult["calibrationResult"].isObject())
			{
				auto obj = m_calibrationResult["calibrationResult"].toObject();
				if (obj.contains("result") && obj["result"].isString() && obj["result"].toString() == "calibrating")
				{
					m_calibrationResult["calibrationResult"] = QJsonObject{{"result", "failed"}, {"error", "previous calibration interrupted"}};
				}
			}
		}
		statusFile.close();
	}
	else
	{
		m_calibrationResult["calibrationResult"] = QJsonObject{{"result", "failed"}, {"error", "not inited"}};
	}
}

PointCloudService::~PointCloudService()
{
	if (m_machineSocket)
	{
		m_machineSocket->disconnectFromHost();
		delete m_machineSocket;
	}
	if (m_workerMachineSocket)
	{
		m_workerMachineSocket->disconnectFromHost();
		delete m_workerMachineSocket;
	}
}

// 保存标定状态到文件
void PointCloudService::saveCalibrationStatus() {
    
    QFile statusFile(m_statusFilePath);
    if (statusFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QByteArray data = QJsonDocument(m_calibrationResult).toJson(QJsonDocument::Indented);
        statusFile.write(data);
        statusFile.close();
    }
}

// 保存工件检查结果到文件
void PointCloudService::savePartInspectResult(const QString& rfid, const QJsonObject& result) {
    
    QString appDir = QCoreApplication::applicationDirPath();
    QString resultDir = appDir + "/PartResult";
    
    // 确保目录存在
    QDir dir(resultDir);
    if (!dir.exists()) {
        dir.mkpath(resultDir);
    }
    
    QString resultFile = resultDir + "/" + rfid + ".json";
    QFile statusFile(resultFile);
    if (statusFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QByteArray data = QJsonDocument(result).toJson(QJsonDocument::Indented);
        statusFile.write(data);
        statusFile.close();
    }
}

// 获取工件检查结果
void PointCloudService::getPartInspectResult(const QJsonObject& params, QTcpSocket* socket, const QString& idCode) {
    // 从 params 中获取 RFID
    QString rfid = params.value("rfid").toString();
    if (rfid.isEmpty()) {
        sendError(socket, "RFID is required", idCode);
        return;
    }
    
    // 构建结果文件路径
    QString appDir = QCoreApplication::applicationDirPath();
    QString resultDir = appDir + "/PartResult";
    QString resultFile = resultDir + "/" + rfid + ".json";
    
    // 检查文件是否存在
    QFile file(resultFile);
    if (!file.exists()) {
        sendError(socket, QString("Inspection result not found for RFID: %1").arg(rfid), idCode);
        return;
    }
    
    // 读取文件内容
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        sendError(socket, QString("Failed to open inspection result file: %1").arg(file.errorString()), idCode);
        return;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    // 解析 JSON
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        sendError(socket, QString("Invalid inspection result file: %1").arg(parseError.errorString()), idCode);
        return;
    }
    
    // 返回结果
    QJsonObject result = doc.object();
    sendResponse(socket, true, "Inspection result retrieved successfully", idCode, result);
}

void PointCloudService::sendRes(QTcpSocket* socket, QJsonObject& resp, const QString& idCode)
{
	if (socket == nullptr)
	{
		return;
	}
	if (!idCode.isEmpty())
		resp["IDCode"] = idCode;

	QByteArray responseBytes = QJsonDocument(resp).toJson(QJsonDocument::Indented) + "\n";

	// 记录发送内容
	CommLogger::instance().logSent(QString::fromUtf8(responseBytes).trimmed());

	socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n");
	socket->flush();
}

void PointCloudService::sendOk(QTcpSocket* socket, const QString& msg, const QString& idCode)
{
	if (socket == nullptr)
	{
		return;
	}
	QJsonObject resp;
	resp["ok"]  = true;
	resp["msg"] = msg;
	if (!idCode.isEmpty())
		resp["IDCode"] = idCode;

	QByteArray responseBytes = QJsonDocument(resp).toJson(QJsonDocument::Indented) + "\n";

	// 记录发送内容
	CommLogger::instance().logSent(QString::fromUtf8(responseBytes).trimmed());

	socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n");
	socket->flush();
}

void PointCloudService::sendError(QTcpSocket* socket, const QString& msg, const QString& idCode)
{
	if (socket == nullptr)
	{
		return;
	}
	QJsonObject resp;
	resp["ok"]  = false;
	resp["msg"] = msg;
	if (!idCode.isEmpty())
		resp["IDCode"] = idCode;

	QByteArray responseBytes = QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n";

	// 记录发送内容
	CommLogger::instance().logSent(QString::fromUtf8(responseBytes).trimmed());

	socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n");
	socket->flush();
}

void PointCloudService::sendResponse(QTcpSocket*        socket,
                                     bool               ok,
                                     const QString&     msg,
                                     const QString&     idCode,
                                     const QJsonObject& extra)
{
	if (socket == nullptr)
	{
		return;
	}

	QJsonObject resp = extra;
	resp["ok"]       = ok;
	resp["msg"]      = msg;
	if (!idCode.isEmpty())
	{
		resp["IDCode"] = idCode;
	}

	QByteArray responseBytes = QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n";
	CommLogger::instance().logSent(QString::fromUtf8(responseBytes).trimmed());

	socket->write(responseBytes);
	socket->flush();
}

ccHObject* PointCloudService::getDbRoot(QTcpSocket* socket, const QString& idCode)
{
	ccHObject* root = m_app->dbRootObject();
	if (!root)
	{
		sendError(socket, "DB root is null", idCode);
	}
	return root;
}

ccHObject* PointCloudService::findByName(ccHObject* node, const QString& name)
{
	if (node->getName() == name)
	{
		return node;
	}
	for (unsigned i = 0; i < node->getChildrenNumber(); ++i)
	{
		if (ccHObject* found = findByName(node->getChild(i), name))
		{
			return found;
		}
	}
	return nullptr;
}

bool PointCloudService::clearDbInternal(QTcpSocket* socket, const QString& idCode)
{
	ccHObject* root = getDbRoot(socket, idCode);
	if (!root)
	{
		return false;
	}

	std::vector<ccHObject*> toDelete;
	toDelete.reserve(root->getChildrenNumber());
	for (unsigned i = 0; i < root->getChildrenNumber(); ++i)
	{
		toDelete.push_back(root->getChild(i));
	}

	for (ccHObject* obj : toDelete)
	{
		m_app->removeFromDB(obj);
	}

	m_app->refreshAll();
	m_app->updateUI();
	m_app->dispToConsole("[TcpPlugin] DB cleared");
	return true;
}

// 返回 buffer 中第一个完整 JSON 对象最后一个 '}' 的下标，找不到返回 -1
int PointCloudService::findJsonObjectEnd(const QByteArray& buffer)
{
	int  depth    = 0;
	int  start    = -1;
	bool inString = false;
	bool escaped  = false;

	for (int i = 0; i < buffer.size(); ++i)
	{
		const char c = buffer[i];

		if (escaped)
		{
			escaped = false;
			continue;
		}
		if (c == '\\' && inString)
		{
			escaped = true;
			continue;
		}
		if (c == '"')
		{
			inString = !inString;
			continue;
		}
		if (inString)
			continue;

		if (c == '{')
		{
			if (depth == 0)
				start = i;
			++depth;
		}
		else if (c == '}')
		{
			--depth;
			if (depth == 0 && start >= 0)
				return i; // 找到完整对象的末尾
		}
	}
	return -1; // 不完整，继续等待
}

bool PointCloudService::ensureConnected(QString* errorMessage, int connectTimeout)
{
	QTcpSocket** socket = nullptr;

	// 根据当前线程选择使用哪个 socket
	if (QThread::currentThread() == this->thread())
	{
		// 主线程使用 m_machineSocket
		socket = &m_machineSocket;
	}
	else
	{
		// 工作线程使用 m_workerMachineSocket
		socket = &m_workerMachineSocket;
	}

	if (*socket && (*socket)->state() == QTcpSocket::ConnectedState)
		return true;

	// 清理旧的 socket
	if (*socket)
	{
		(*socket)->abort();
		(*socket)->deleteLater();
		*socket = nullptr;
	}

	*socket = new QTcpSocket(this);
	(*socket)->connectToHost(MACHINE_HOST, MACHINE_PORT);
	if (!(*socket)->waitForConnected(connectTimeout))
	{
		setError(errorMessage,
		         "Failed to connect machine middleware: " + (*socket)->errorString());
		(*socket)->abort();
		(*socket)->deleteLater();
		*socket = nullptr;
		return false;
	}
	return true;
}

void PointCloudService::resetConnection()
{
	if (m_machineSocket)
	{
		m_machineSocket->abort();
		m_machineSocket->deleteLater();
		m_machineSocket = nullptr;
	}
	if (m_workerMachineSocket)
	{
		m_workerMachineSocket->abort();
		m_workerMachineSocket->deleteLater();
		m_workerMachineSocket = nullptr;
	}
}

void PointCloudService::setError(QString* out, const QString& msg)
{
	if (out)
		*out = msg;
}

bool PointCloudService::sendMachineCommand(const QJsonObject& params, QJsonObject& response, QString* errorMessage, int timeout)
{
	if (!ensureConnected(errorMessage, timeout))
		return false;

	// 根据当前线程选择使用哪个 socket
	QTcpSocket* socket = nullptr;
	if (QThread::currentThread() == this->thread())
	{
		socket = m_machineSocket;
	}
	else
	{
		socket = m_workerMachineSocket;
	}

	// --- 构造命令，生成 IDCode ---
	QJsonObject   command = params;
	const QString idCode  = command.contains("IDCode")
	                            ? command["IDCode"].toString()
	                            : QUuid::createUuid().toString(QUuid::WithoutBraces);
	command["IDCode"]     = idCode;

	// --- 发送前清空 socket 残留数据（处理上次超时遗留）---
	if (socket->bytesAvailable() > 0)
	{
		socket->readAll();
	}

	const QByteArray payload = QJsonDocument(command).toJson(QJsonDocument::Compact);
	if (socket->write(payload) == -1 || !socket->waitForBytesWritten(timeout))
	{
		setError(errorMessage,
		         "Failed to send machine command: " + socket->errorString());
		resetConnection();
		return false;
	}

	// --- 接收，匹配 IDCode，丢弃不匹配的包 ---
	QElapsedTimer timer;
	timer.start();
	QByteArray buffer;

	while (timer.elapsed() < timeout)
	{
		const int remain = timeout - static_cast<int>(timer.elapsed());
		if (remain <= 0)
			break;

		if (socket->bytesAvailable() == 0 && !socket->waitForReadyRead(remain))
		{
			continue;
		}

		buffer.append(socket->readAll());

		// 从 buffer 中提取所有完整 JSON 对象，逐个检查 IDCode
		while (true)
		{
			const int jsonEnd = findJsonObjectEnd(buffer);
			if (jsonEnd < 0)
				break; // 还没有完整的包，等待更多数据

			const QByteArray jsonBytes = buffer.left(jsonEnd + 1);
			buffer.remove(0, jsonEnd + 1); // 消费掉这个包

			QJsonParseError err;
			QJsonDocument   doc = QJsonDocument::fromJson(jsonBytes, &err);

			if (err.error != QJsonParseError::NoError || !doc.isObject())
			{
				continue;
			}

			const QJsonObject obj        = doc.object();
			const QString     receivedId = obj["IDCode"].toString();

			if (receivedId != idCode)
			{
				// IDCode 不匹配，是上一个请求的残留，丢弃
				continue;
			}

			// 匹配成功
			response = obj;
			return true;
		}
	}

	setError(errorMessage, "Machine command response timeout");
	return false;
}

bool PointCloudService::checkMachineCommandRet(const QJsonObject& response,
                                               const QString&     commandName,
                                               QString*           errorMessage,
                                               const QString&     messageKey)
{
	const QString retKey = commandName + "_Ret";
	if (response.contains(retKey) && response[retKey].toString() == "0")
	{
		return true;
	}

	if (errorMessage)
	{
		const QString preferredKey = messageKey.isEmpty() ? (commandName + "_message") : messageKey;
		QString       detail       = response.value(preferredKey).toString();
		if (detail.isEmpty())
		{
			detail = response.value("Msg").toString();
		}
		if (detail.isEmpty())
		{
			detail = response.value("message").toString();
		}
		if (detail.isEmpty())
		{
			detail = QString("Machine command failed: %1").arg(commandName);
		}
		*errorMessage = detail;
	}
	return false;
}

bool PointCloudService::sendFileToMachine(const QString& filePath, QString* errorMessage)
{
	const int   timeout = 5;
	QJsonObject params;
	params["Command"]    = "SendFile";
	params["DeviceName"] = MACHINE_DEVICE_NAME;
	params["DeviceType"] = MACHINE_DEVICE_TYPE;
	params["CNCPath"]    = CALIBRATION_CNC_PATH;
	params["CNCFile"]    = CALIBRATION_CNC_FILE;
	params["LocalFile"]  = filePath;
	params["Timeout"]    = timeout * 1000;

	QJsonObject response;
	return sendMachineCommand(params, response, errorMessage, timeout * 1000)
	       && checkMachineCommandRet(response, "SendFile", errorMessage);
}

bool PointCloudService::getMachineMode(QString& mode, QString* errorMessage)
{
	const int   timeout = 5;
	QJsonObject params;
	params["Command"]    = "GetDeviceMode";
	params["DeviceType"] = MACHINE_DEVICE_TYPE;
	params["DeviceName"] = MACHINE_DEVICE_NAME;
	params["Timeout"]    = timeout * 1000;

	QJsonObject response;
	if (!sendMachineCommand(params, response, errorMessage, timeout * 1000))
	{
		return false;
	}
	if (!checkMachineCommandRet(response, "GetDeviceMode", errorMessage))
	{
		return false;
	}

	mode = response["Msg"].toString();
	return true;
}

bool PointCloudService::setMainProgram(QString* errorMessage)
{
	const int   timeout = 5;
	QJsonObject params;
	params["Command"]    = "SetMainProg";
	params["DeviceType"] = MACHINE_DEVICE_TYPE;
	params["DeviceName"] = MACHINE_DEVICE_NAME;
	params["MainProg"]   = CALIBRATION_CNC_FILE;
	params["Path"]       = CALIBRATION_CNC_PATH;
	params["Timeout"]    = timeout * 1000;

	QJsonObject response;
	return sendMachineCommand(params, response, errorMessage, timeout * 1000)
	       && checkMachineCommandRet(response, "SetMainProg", errorMessage);
}

bool PointCloudService::startMachine(QString* errorMessage)
{
	// 尝试多几次
	int retryCount = 0;
	while (retryCount++ < 20)
	{
		const int   timeout = 5;
		QJsonObject params;
		params["Command"]    = "WritePlc";
		params["DeviceType"] = MACHINE_DEVICE_TYPE;
		params["DeviceName"] = MACHINE_DEVICE_NAME;
		params["Addr"]       = "999";
		params["AddrType"]   = "R";
		params["Bit"]        = "0";
		params["Timeout"]    = timeout * 1000;

		QJsonObject response;
		params["Value"] = "1";
		if (!sendMachineCommand(params, response, errorMessage, timeout * 1000)
		    || !checkMachineCommandRet(response, "WritePlc", errorMessage))
		{

			QThread::msleep(200);
			continue;
		}

		QThread::msleep(200);

		params["Value"] = "0";
		if (!sendMachineCommand(params, response, errorMessage, timeout * 1000)
		    || !checkMachineCommandRet(response, "WritePlc", errorMessage))
		{
			continue;
		}

		QThread::msleep(1000);
		QString value;
		if (!getDeviceRun(value, errorMessage))
		{
			continue;
		}

		if (value == "2")
		{
			return true;
		}
	}

	return false;
}

bool PointCloudService::getDeviceRun(QString& value, QString* errorMessage)
{
	const int   timeout = 5;
	QJsonObject params;
	params["Command"]    = "GetDeviceRun";
	params["DeviceName"] = MACHINE_DEVICE_NAME;
	params["DeviceType"] = MACHINE_DEVICE_TYPE;
	params["Timeout"]    = timeout * 1000;

	QJsonObject response;
	QString     currentError;

	if (!sendMachineCommand(params, response, &currentError, timeout * 1000))
	{
		return false;
	}

	if (!checkMachineCommandRet(response, "GetDeviceRun", errorMessage))
	{
		return false;
	}

	if (!response.contains("Value"))
	{
		return false;
	}

	value = response["Value"].toString();

	return true;
}

bool PointCloudService::waitForMachineIdle(int timeoutSeconds, QString* errorMessage)
{
	const int maxWaitTime  = timeoutSeconds * 1000;
	const int waitInterval = 50;
	int       elapsedTime  = 0;

	while (elapsedTime < maxWaitTime)
	{
		QString value;
		if (!getDeviceRun(value, errorMessage))
		{
			continue;
		}
		if (value == "0")
		{
			return true;
		}

		// 用 QEventLoop 替代 sleep，保持UI响应
		QEventLoop loop;
		QTimer::singleShot(waitInterval, &loop, &QEventLoop::quit);
		loop.exec();

		elapsedTime += waitInterval;
	}

	if (errorMessage && errorMessage->isEmpty())
	{
		*errorMessage = "Wait for machine idle timeout";
	}
	return false;
}

QVector<QVector3D> PointCloudService::resolveCalibrationPositions(const QJsonObject& params, QString* errorMessage) const
{
	const QJsonValue positionsValue = params.value("positions");
	if (positionsValue.isUndefined() || positionsValue.isNull())
	{
		return CalibrationDialog::getDefaultPositions();
	}

	if (!positionsValue.isArray())
	{
		if (errorMessage)
		{
			*errorMessage = "'positions' must be an array";
		}
		return {};
	}

	const QJsonArray positionsArray = positionsValue.toArray();
	if (positionsArray.isEmpty())
	{
		return CalibrationDialog::getDefaultPositions();
	}

	QVector<QVector3D> positions;
	positions.reserve(positionsArray.size());

	for (int i = 0; i < positionsArray.size(); ++i)
	{
		const QJsonValue entry = positionsArray.at(i);
		if (entry.isObject())
		{
			const QJsonObject obj = entry.toObject();
			if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z"))
			{
				if (errorMessage)
				{
					*errorMessage = QString("positions[%1] is missing x/y/z").arg(i);
				}
				return {};
			}
			positions.push_back(QVector3D(static_cast<float>(obj["x"].toDouble()),
			                              static_cast<float>(obj["y"].toDouble()),
			                              static_cast<float>(obj["z"].toDouble())));
			continue;
		}

		if (entry.isArray())
		{
			const QJsonArray arr = entry.toArray();
			if (arr.size() < 3)
			{
				if (errorMessage)
				{
					*errorMessage = QString("positions[%1] must contain at least 3 values").arg(i);
				}
				return {};
			}
			positions.push_back(QVector3D(static_cast<float>(arr.at(0).toDouble()),
			                              static_cast<float>(arr.at(1).toDouble()),
			                              static_cast<float>(arr.at(2).toDouble())));
			continue;
		}

		if (errorMessage)
		{
			*errorMessage = QString("positions[%1] must be an object or array").arg(i);
		}
		return {};
	}

	return positions;
}

PointCloudService::CalibrationRigidTransform PointCloudService::computeRigidTransform(
    const std::vector<Eigen::Vector3d>& scanner_points,
    const std::vector<Eigen::Vector3d>& machine_points)
{
	const size_t count = scanner_points.size();
	if (count < 3 || count != machine_points.size())
	{
		throw std::runtime_error("At least 3 corresponding calibration points are required");
	}

	Eigen::Vector3d scannerCenter = Eigen::Vector3d::Zero();
	Eigen::Vector3d machineCenter = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < count; ++i)
	{
		scannerCenter += scanner_points[i];
		machineCenter += machine_points[i];
	}
	scannerCenter /= static_cast<double>(count);
	machineCenter /= static_cast<double>(count);

	Eigen::MatrixXd scannerOffset(3, count);
	Eigen::MatrixXd machineOffset(3, count);
	for (size_t i = 0; i < count; ++i)
	{
		scannerOffset.col(i) = scanner_points[i] - scannerCenter;
		machineOffset.col(i) = machine_points[i] - machineCenter;
	}

	const Eigen::Matrix3d             covariance = scannerOffset * machineOffset.transpose();
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d                   rotation = svd.matrixV() * svd.matrixU().transpose();
	if (rotation.determinant() < 0)
	{
		Eigen::Matrix3d V = svd.matrixV();
		V.col(2) *= -1;
		rotation = V * svd.matrixU().transpose();
	}

	CalibrationRigidTransform transform;
	transform.R = rotation;
	transform.T = machineCenter - rotation * scannerCenter;
	return transform;
}

Eigen::Matrix4d PointCloudService::toMatrix4d(const CalibrationRigidTransform& tf)
{
	Eigen::Matrix4d matrix   = Eigen::Matrix4d::Identity();
	matrix.block<3, 3>(0, 0) = tf.R;
	matrix.block<3, 1>(0, 3) = tf.T;
	return matrix;
}

void PointCloudService::load(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString path = params["path"].toString();
        if (path.isEmpty()) {
            sendError(socket, "Empty path", idCode);
            return;
        }

        FileIOFilter::Shared filter = FileIOFilter::FindBestFilterForExtension(QFileInfo(path).suffix());
        if (!filter) {
            m_app->dispToConsole("[TcpPlugin] Unsupported file format", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
            sendError(socket, "Unsupported file format", idCode);
            return;
        }

        ccHObject* container = new ccHObject();
        if (filter->loadFile(path, *container, FileIOFilter::LoadParameters()) == CC_FERR_NO_ERROR) {
            const QString modelName = params["name"].toString();
            if (!modelName.isEmpty()) {
                container->setName(modelName);
            }

            m_app->addToDB(container);
            m_app->dispToConsole("[TcpPlugin] Loaded: " + path);
            sendOk(socket, "Loaded: " + path, idCode);
        } else {
            m_app->dispToConsole("[TcpPlugin] Failed to load: " + path, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
            sendError(socket, "Failed to load: " + path, idCode);
            delete container;
        } },
	                          Qt::QueuedConnection);
}

void PointCloudService::filter(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, socket, idCode]()
	                          { sendError(socket, "Filter not implemented", idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::icp(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString dataName = params["data"].toString();
        const QString modelName = params["model"].toString();

        if (dataName.isEmpty() || modelName.isEmpty()) {
            sendError(socket, "Missing 'data' or 'model' parameter", idCode);
            return;
        }

        // Build ICP parameters with defaults
        CCCoreLib::ICPRegistrationTools::Parameters icpParams;
        icpParams.convType = CCCoreLib::ICPRegistrationTools::CONVERGENCE_TYPE::MAX_ERROR_CONVERGENCE;
        icpParams.minRMSDecrease = params["minRMSDecrease"].toDouble(1e-5);
        icpParams.nbMaxIterations = static_cast<unsigned>(params["maxIterations"].toInt(20));
        icpParams.adjustScale = params["adjustScale"].toBool(false);
        icpParams.filterOutFarthestPoints = params["removeFarthestPoints"].toBool(false);
        icpParams.samplingLimit = static_cast<unsigned>(params["samplingLimit"].toInt(50000));
        icpParams.finalOverlapRatio = params["finalOverlap"].toDouble(1.0);
        icpParams.transformationFilters = CCCoreLib::RegistrationTools::SKIP_NONE;
        icpParams.maxThreadCount = params["maxThreadCount"].toInt(0);
        icpParams.useC2MSignedDistances = false;
        icpParams.robustC2MSignedDistances = true;
        icpParams.normalsMatching = CCCoreLib::ICPRegistrationTools::NO_NORMAL;

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        // Find a point cloud or mesh by name, searching parent first then children
        auto findCloudOrMesh = [&](const QString& name, const QString& role) -> ccHObject* {
            ccHObject* parent = findByName(root, name);
            if (!parent) {
                sendError(socket, role + " object not found: " + name, idCode);
                return nullptr;
            }
            for (unsigned i = 0; i < parent->getChildrenNumber(); ++i) {
                ccHObject* child = parent->getChild(i);
                if (child->isKindOf(CC_TYPES::POINT_CLOUD) || child->isKindOf(CC_TYPES::MESH)) {
                    return child;
                }
            }
            return parent;
        };

        ccHObject* dataObj = findCloudOrMesh(dataName, "Data");
        if (!dataObj) return;
        ccHObject* modelObj = findCloudOrMesh(modelName, "Model");
        if (!modelObj) return;

        if ((!dataObj->isKindOf(CC_TYPES::POINT_CLOUD) && !dataObj->isKindOf(CC_TYPES::MESH)) ||
            (!modelObj->isKindOf(CC_TYPES::POINT_CLOUD) && !modelObj->isKindOf(CC_TYPES::MESH))) {
            sendError(socket, "Both objects must be point clouds or meshes", idCode);
            return;
        }

        // Run ICP
        ccGLMatrix transMat;
        double finalError = 0.0;
        double finalScale = 1.0;
        unsigned finalPointCount = 0;

        bool success = ccRegistrationTools::ICP(
            dataObj, modelObj, transMat, finalScale, finalError, finalPointCount,
            icpParams, false, false, (QWidget*)(m_app->getMainWindow()));

        if (!success) {
            sendError(socket, "ICP failed", idCode);
            return;
        }

        // Retrieve point cloud to transform
        ccGenericPointCloud* pc = nullptr;
        if (dataObj->isKindOf(CC_TYPES::POINT_CLOUD)) {
            pc = ccHObjectCaster::ToGenericPointCloud(dataObj);
        } else if (dataObj->isKindOf(CC_TYPES::MESH)) {
            ccGenericMesh* mesh = ccHObjectCaster::ToGenericMesh(dataObj);
            pc = mesh->getAssociatedCloud();
            if (pc && pc->isLocked()) {
                sendError(socket, "Mesh vertices are locked, cannot apply transformation", idCode);
                return;
            }
        }

        if (!pc) {
            sendError(socket, "Failed to get point cloud from data object", idCode);
            return;
        }

        if (dataObj->isAncestorOf(modelObj)) {
            pc->applyRigidTransformation(transMat);
        } else {
            pc->applyGLTransformation_recursive(&transMat);
        }

        if (dataObj->isKindOf(CC_TYPES::MESH)) {
            ccHObjectCaster::ToGenericMesh(dataObj)->refreshBB();
        }

        // Sync global shift from model
        ccGenericPointCloud* refPC = ccHObjectCaster::ToGenericPointCloud(modelObj);
        if (refPC && refPC->isShifted()) {
            const CCVector3d& Pshift = refPC->getGlobalShift();
            const double scale = refPC->getGlobalScale();
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
        result["finalRMS"] = finalError;
        result["finalPointCount"] = static_cast<int>(finalPointCount);
        result["finalScale"] = finalScale;
        result["matrix"] = matrixStr;
        sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact), idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::camera(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        auto* glWindow = m_app->getActiveGLWindow();
        if (!glWindow) {
            sendError(socket, "No active GL window", idCode);
            return;
        }

        if (!params.contains("viewPreset")) {
            sendError(socket, "Missing viewPreset parameter", idCode);
            return;
        }

        const QString preset = params["viewPreset"].toString();
        if (preset == "top") glWindow->setView(CC_TOP_VIEW);
        else if (preset == "front") glWindow->setView(CC_FRONT_VIEW);
        else if (preset == "iso") glWindow->setView(CC_ISO_VIEW_1);

        glWindow->redraw();
        sendOk(socket, "Camera view set to: " + preset, idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::applyViewport(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString name = params["name"].toString();
        if (name.isEmpty()) {
            sendError(socket, "Empty name parameter", idCode);
            return;
        }

        ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
        if (!glWindow) {
            sendError(socket, "No active GL window", idCode);
            return;
        }

        // Search only top-level children
        ccHObject* rootObject = nullptr;
        ccHObject* dbRoot = m_app->dbRootObject();
        if (dbRoot) {
            for (unsigned i = 0; i < dbRoot->getChildrenNumber(); ++i) {
                ccHObject* child = dbRoot->getChild(i);
                if (child && child->getName() == name) {
                    rootObject = child->getChild(0);
                    break;
                }
            }
        }

        if (!rootObject) {
            m_app->dispToConsole("[TcpPlugin] Object not found: " + name, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
            sendError(socket, "Object not found: " + name, idCode);
            return;
        }

        cc2DViewportObject* viewportObject = nullptr;
        for (unsigned i = 0; i < rootObject->getChildrenNumber(); ++i) {
            ccHObject* obj = rootObject->getChild(i);
            if (obj && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT)) {
                viewportObject = static_cast<cc2DViewportObject*>(obj);
                break;
            }
        }

        if (!viewportObject) {
            m_app->dispToConsole("[TcpPlugin] Viewport object not found in hierarchy", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
            sendError(socket, "Viewport object not found in hierarchy", idCode);
            return;
        }

        cc2DViewportObject* viewport = ccHObjectCaster::To2DViewportObject(viewportObject);
        assert(viewport);
        if (!viewport) {
            return;
        }

        glWindow->setViewportParameters(viewport->getParameters());
        glWindow->redraw();
        sendOk(socket, "Viewport applied", idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::applyTransformation(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString objectName = params["name"].toString();
        const QString matrixText = params["matrix"].toString();
        const bool applyToGlobal = params["applyToGlobal"].toBool(false);
        const bool inverse = params["inverse"].toBool(false);

        if (objectName.isEmpty()) { sendError(socket, "Missing 'name' parameter", idCode); return; }
        if (matrixText.isEmpty()) { sendError(socket, "Missing 'matrix' parameter", idCode); return; }

        bool valid = false;
        ccGLMatrixd mat = ccGLMatrixd::FromString(matrixText, valid);
        if (!valid) {
            sendError(socket, "Invalid matrix format", idCode);
            return;
        }
        if (inverse) {
            mat.invert();
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        ccHObject* entity = findByName(root, objectName);
        if (!entity) {
            sendError(socket, "Object not found: " + objectName, idCode);
            return;
        }

        // Compute the actual local transformation matrix
        ccGLMatrixd transMat = mat;
        if (applyToGlobal) {
            ccShiftedObject* shiftedEntity = dynamic_cast<ccShiftedObject*>(entity);
            if (shiftedEntity) {
                CCVector3d globalShift = shiftedEntity->getGlobalShift();
                double globalScale = shiftedEntity->getGlobalScale();

                CCVector3d rotatedGlobalShift = globalShift;
                mat.applyRotation(rotatedGlobalShift);
                CCVector3d localTranslation = globalScale * (globalShift - rotatedGlobalShift + mat.getTranslationAsVec3D());
                transMat.setTranslation(localTranslation);
            }
        }

        // Check for locked vertices
        bool lockedVertices = false;
        ccGenericPointCloud* cloud = ccHObjectCaster::ToGenericPointCloud(entity, &lockedVertices);
        if (cloud && lockedVertices) {
            sendError(socket, "Vertices are locked: " + objectName, idCode);
            return;
        }
        if (cloud) {
            cloud->deleteOctree();
        }

        entity->setGLTransformation(ccGLMatrix(transMat.data()));
        entity->applyGLTransformation_recursive();
        entity->prepareDisplayForRefresh_recursive();

        m_app->updateUI();
        m_app->refreshAll();

        m_app->dispToConsole("[TcpPlugin] Transformation applied to: " + objectName);
        sendOk(socket, "Transformation applied to: " + objectName, idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::segment(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString meshName = params["meshName"].toString();
        const QString binName = params["binName"].toString();
        const bool keepInside = params["keepInside"].toBool(true);
        const bool modifySource = params["modifySource"].toBool(false);
        const QString outputName = params["outputName"].toString();

        if (meshName.isEmpty() || binName.isEmpty()) {
            sendError(socket, "Missing meshName or binName", idCode);
            return;
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        // Find target object (mesh or point cloud)
        ccHObject* targetParent = findByName(root, meshName);
        if (!targetParent) {
            sendError(socket, "Object not found: " + meshName, idCode);
            return;
        }
        ccHObject* targetObj = targetParent->getChild(0);
        if (!targetObj) {
            sendError(socket, "Object has no children: " + meshName, idCode);
            return;
        }

        ccMesh* targetMesh = nullptr;
        ccGenericPointCloud* targetCloud = nullptr;
        if (targetObj->isKindOf(CC_TYPES::MESH)) {
            targetMesh = ccHObjectCaster::ToMesh(targetObj);
        } else if (targetObj->isKindOf(CC_TYPES::POINT_CLOUD)) {
            targetCloud = ccHObjectCaster::ToGenericPointCloud(targetObj);
        } else {
            sendError(socket, "Object is not a mesh or point cloud: " + meshName, idCode);
            return;
        }

        // Find bin and extract polyline + viewport from its hierarchy
        ccHObject* binRoot = findByName(root, binName);
        if (!binRoot) {
            sendError(socket, "Bin root not found: " + binName, idCode);
            return;
        }

        ccPolyline* segPoly = nullptr;
        cc2DViewportObject* vpObject = nullptr;
        std::function<void(ccHObject*)> findInHierarchy = [&](ccHObject* obj) {
            if (!segPoly && obj->isKindOf(CC_TYPES::POLY_LINE)) {
                segPoly = static_cast<ccPolyline*>(obj);
            }
            if (!vpObject && obj->isKindOf(CC_TYPES::VIEWPORT_2D_OBJECT)) {
                vpObject = static_cast<cc2DViewportObject*>(obj);
            }
            for (unsigned i = 0; i < obj->getChildrenNumber(); ++i) {
                findInHierarchy(obj->getChild(i));
            }
        };
        findInHierarchy(binRoot);

        if (!segPoly) { sendError(socket, "Polyline not found in bin", idCode); return; }
        if (!segPoly->isClosed()) { sendError(socket, "Polyline is not closed", idCode); return; }
        if (!vpObject) { sendError(socket, "Viewport object not found in bin", idCode); return; }

        // Apply viewport and capture camera
        ccGLWindowInterface* glWindow = m_app->getActiveGLWindow();
        if (!glWindow) {
            sendError(socket, "No active GL window", idCode);
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
        ccPolyline* segPoly2D = new ccPolyline(polyVertices2D);
        segPoly2D->addChild(polyVertices2D);

        CCCoreLib::GenericIndexedCloudPersist* vertices = segPoly->getAssociatedCloud();
        const bool mode3D = !segPoly->is2DMode();

        if (!polyVertices2D->reserve(vertices->size()) || !segPoly2D->reserve(segPoly->size())) {
            delete segPoly2D;
            sendError(socket, "Not enough memory for polyline conversion", idCode);
            return;
        }

        for (unsigned i = 0; i < vertices->size(); ++i) {
            CCVector3 P = *vertices->getPoint(i);
            if (mode3D) {
                CCVector3d Q2D;
                camera.project(P, Q2D);
                P.x = static_cast<PointCoordinateType>(Q2D.x - half_w);
                P.y = static_cast<PointCoordinateType>(Q2D.y - half_h);
                P.z = 0;
            }
            polyVertices2D->addPoint(P);
        }
        for (unsigned j = 0; j < segPoly->size(); ++j) {
            segPoly2D->addPointIndex(segPoly->getPointGlobalIndex(j));
        }
        segPoly2D->setClosed(segPoly->isClosed());

        // Check if polygon is fully inside viewport
        bool polyInsideViewport = true;
        for (unsigned i = 0; i < segPoly2D->size(); ++i) {
            const CCVector3* P2D = segPoly2D->getPoint(i);
            if (P2D->x < -half_w || P2D->x > half_w || P2D->y < -half_h || P2D->y > half_h) {
                polyInsideViewport = false;
                break;
            }
        }

        // Resolve the point cloud to classify
        ccGenericPointCloud* cloud = targetMesh ? ccHObjectCaster::ToGenericPointCloud(targetMesh) : targetCloud;
        if (targetMesh && !cloud) {
            delete segPoly2D;
            sendError(socket, "Mesh has no associated point cloud", idCode);
            return;
        }

        if (!cloud->isVisibilityTableInstantiated() && !cloud->resetVisibilityArray()) {
            delete segPoly2D;
            sendError(socket, "Failed to initialize visibility array", idCode);
            return;
        }

        // Project and classify each point against the polygon
        ccGenericPointCloud::VisibilityTableType& visibilityArray = cloud->getTheVisibilityArray();
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            if (visibilityArray[i] != CCCoreLib::POINT_VISIBLE) {
                continue;
            }

            CCVector3d Q2D;
            bool pointInFrustum = false;
            camera.project(*cloud->getPoint(i), Q2D, &pointInFrustum);

            bool pointInside = false;
            if (pointInFrustum || !polyInsideViewport) {
                CCVector2 P2D(static_cast<PointCoordinateType>(Q2D.x - half_w),
                              static_cast<PointCoordinateType>(Q2D.y - half_h));
                pointInside = CCCoreLib::ManualSegmentationTools::isPointInsidePoly(P2D, segPoly2D);
            }

            visibilityArray[i] = (keepInside != pointInside) ? CCCoreLib::POINT_HIDDEN : CCCoreLib::POINT_VISIBLE;
        }

        // Generate result object
        auto finishWithEmpty = [&]() {
            delete segPoly2D;
            cloud->resetVisibilityArray();
            sendError(socket, "Segmentation result is empty", idCode);
        };

        if (targetMesh) {
            ccMesh* result = targetMesh->createNewMeshFromSelection(modifySource);
            if (!result || result->size() == 0) { delete result; finishWithEmpty(); return; }

            const QString resultName = outputName.isEmpty() ? targetMesh->getName() + "_segmented" : outputName;
            result->setName(resultName);
            m_app->addToDB(result);
            if (modifySource) {
                targetMesh->prepareDisplayForRefresh_recursive();
            }

            cloud->resetVisibilityArray();
            delete segPoly2D;
            m_app->refreshAll();

            const QString msg = modifySource ? QString("Source mesh punched, new part: ") + resultName
                                           : QString("Segmentation completed: ") + resultName;
            m_app->dispToConsole("[TcpPlugin] " + msg);
            sendOk(socket, msg, idCode);
        } else {
            ccGenericPointCloud* result = targetCloud->createNewCloudFromVisibilitySelection(modifySource);
            if (!result || result->size() == 0) { delete result; finishWithEmpty(); return; }

            const QString resultName = outputName.isEmpty() ? targetCloud->getName() + "_segmented" : outputName;
            result->setName(resultName);
            m_app->addToDB(result);
            if (modifySource) {
                targetCloud->prepareDisplayForRefresh_recursive();
            }

            cloud->resetVisibilityArray();
            delete segPoly2D;
            m_app->refreshAll();

            const QString msg = modifySource ? QString("Source cloud punched, removed part: ") + resultName
                                           : QString("Segmentation completed: ") + resultName;
            m_app->dispToConsole("[TcpPlugin] " + msg);
            sendOk(socket, msg, idCode);
        } },
	                          Qt::QueuedConnection);
}

void PointCloudService::deleteObject(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString objectName = params["name"].toString();
        if (objectName.isEmpty()) {
            sendError(socket, "Missing 'name' parameter", idCode);
            return;
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        ccHObject* target = findByName(root, objectName);
        if (!target) {
            sendError(socket, "Object not found: " + objectName, idCode);
            return;
        }

        m_app->removeFromDB(target);
        m_app->refreshAll();

        m_app->dispToConsole("[TcpPlugin] Deleted: " + objectName);
        sendOk(socket, "Deleted: " + objectName, idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::fit(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString type = params["type"].toString();
        if (type == "sphere") {
			double centerX, centerY, centerZ, rms;
			handleFitSphere(params, socket, idCode, centerX, centerY, centerZ, rms);
        } else {
            sendError(socket, "Unknown fit type: " + type, idCode);
        } },
	                          Qt::QueuedConnection);
}

bool PointCloudService::handleFitSphere(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, double& centerX, double& centerY, double& centerZ, double& outRms)
{
	const QString objectName       = params["name"].toString();
	const double  outliersRatio    = params["outliersRatio"].toDouble(0.5);
	const double  confidence       = params["confidence"].toDouble(0.99);
	const bool    autoDetectRadius = params["autoDetectRadius"].toBool(true);
	const double  radius           = params["radius"].toDouble(0.0);
	const double  rmsThreshold     = params["rms"].toDouble(-1.0);
	const int     maxRetries       = params["retries"].toInt(0);

	if (objectName.isEmpty())
	{
		sendError(socket, "Missing 'name' parameter", idCode);
		return false;
	}

	ccHObject* root = getDbRoot(socket, idCode);
	if (!root)
	{
		return false;
	}

	// Find a point cloud by name (direct match or first child)
	auto findPointCloud = [&](const QString& name) -> ccPointCloud*
	{
		ccHObject* found = findByName(root, name);
		if (!found)
		{
			return nullptr;
		}
		if (found->isA(CC_TYPES::POINT_CLOUD))
		{
			return static_cast<ccPointCloud*>(found);
		}
		for (unsigned i = 0; i < found->getChildrenNumber(); ++i)
		{
			if (found->getChild(i)->isA(CC_TYPES::POINT_CLOUD))
			{
				return static_cast<ccPointCloud*>(found->getChild(i));
			}
		}
		return nullptr;
	};

	ccPointCloud* cloud = findPointCloud(objectName);
	if (!cloud)
	{
		sendError(socket, "Point cloud not found: " + objectName, idCode);

		return false;
	}

	// Run sphere fitting with retry logic
	CCVector3           center;
	PointCoordinateType fitRadius  = autoDetectRadius ? 0 : static_cast<PointCoordinateType>(radius);
	double              rms        = std::numeric_limits<double>::quiet_NaN();
	int                 retryCount = 0;
	bool                success    = false;
	int                 fitResult  = CCCoreLib::GeometricalAnalysisTools::NoError;

	while (retryCount <= maxRetries)
	{
		fitResult = CCCoreLib::GeometricalAnalysisTools::DetectSphereRobust(
		    cloud, outliersRatio, center, fitRadius, rms, !autoDetectRadius, nullptr, confidence);

		if (fitResult != CCCoreLib::GeometricalAnalysisTools::NoError)
		{
			break;
		}

		// Check if RMS threshold is set and if current RMS meets the threshold
		if (rmsThreshold > 0 && rms >= rmsThreshold)
		{
			retryCount++;
			if (retryCount > maxRetries)
			{
				// All retries failed
				sendError(socket, QString("Sphere fitting failed to meet RMS threshold after %1 retries").arg(maxRetries), idCode);

				return false;
			}
			// Reset for next retry
			rms       = std::numeric_limits<double>::quiet_NaN();
			fitRadius = autoDetectRadius ? 0 : static_cast<PointCoordinateType>(radius);
			continue;
		}

		success = true;
		break;
	}

	if (!success || fitResult != CCCoreLib::GeometricalAnalysisTools::NoError)
	{
		sendError(socket, QString("Sphere fitting failed on '%1' (error code: %2)").arg(objectName).arg(fitResult), idCode);
		return false;
	}

	m_app->dispToConsole(
	    QString("[TcpPlugin][FitSphere] Cloud '%1': center (%2,%3,%4) - radius = %5 [RMS = %6] (retries: %7)")
	        .arg(cloud->getName())
	        .arg(center.x)
	        .arg(center.y)
	        .arg(center.z)
	        .arg(fitRadius)
	        .arg(rms)
	        .arg(retryCount));

	// Create sphere object and add to DB
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
	resultJson["retries"]  = retryCount;
	sendOk(socket, QJsonDocument(resultJson).toJson(QJsonDocument::Compact), idCode);

	centerX = center.x;
	centerY = center.y;
	centerZ = center.z;
	outRms  = rms;

	return true;
}

void PointCloudService::clearDB(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	sendOk(socket, "DB cleared Start", idCode);

	auto id = QThread::currentThreadId();
	QMetaObject::invokeMethod(qApp, [this, socket, idCode]()
	                          {
		
	auto        id     = QThread::currentThreadId();
        if (!clearDbInternal(socket, idCode)) {
            return;
        }
        sendOk(socket, "DB cleared", idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::subsample(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString objectName = params["name"].toString();
        const QString method = params["method"].toString("random");
        const QString outputName = params["outputName"].toString();

        if (objectName.isEmpty()) {
            sendError(socket, "Missing 'name' parameter", idCode);
            return;
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        // 查找点云（直接匹配或取第一个子节点）
        ccPointCloud* cloud = nullptr;
        {
            ccHObject* found = findByName(root, objectName);
            if (!found) {
                sendError(socket, "Object not found: " + objectName, idCode);
                return;
            }
            if (found->isA(CC_TYPES::POINT_CLOUD)) {
                cloud = static_cast<ccPointCloud*>(found);
            } else {
                for (unsigned i = 0; i < found->getChildrenNumber(); ++i) {
                    if (found->getChild(i)->isA(CC_TYPES::POINT_CLOUD)) {
                        cloud = static_cast<ccPointCloud*>(found->getChild(i));
                        break;
                    }
                }
            }
        }

        if (!cloud) {
            sendError(socket, "Point cloud not found: " + objectName, idCode);
            return;
        }

        // 执行采样
        CCCoreLib::ReferenceCloud* sampledRef = nullptr;

        if (method == "random") {
            // parameter: 目标保留点数
            const unsigned targetCount = static_cast<unsigned>(params["parameter"].toInt(10000));
            if (targetCount == 0 || targetCount >= cloud->size()) {
                sendError(socket, "Invalid parameter for random subsampling", idCode);
                return;
            }
            sampledRef = CCCoreLib::CloudSamplingTools::subsampleCloudRandomly(cloud, targetCount);
        } else if (method == "spatial") {
            // parameter: 最小点间距
            const double minDist = params["parameter"].toDouble(0.0);
            if (minDist <= 0.0) {
                sendError(socket, "Invalid parameter for spatial subsampling: minDist must be > 0", idCode);
                return;
            }
            CCCoreLib::CloudSamplingTools::SFModulationParams modParams;
            modParams.enabled = false;
            sampledRef = CCCoreLib::CloudSamplingTools::resampleCloudSpatially(
                cloud,
                static_cast<PointCoordinateType>(minDist),
                modParams);
        } else if (method == "octree") {
            // parameter: octree 细分等级 (1~21)
            const int level = params["parameter"].toInt(8);
            if (level < 1 || level > 21) {
                sendError(socket, "Invalid parameter for octree subsampling: level must be 1~21", idCode);
                return;
            }
            ccOctree::Shared octree = cloud->getOctree();
            if (!octree) {
                octree = cloud->computeOctree();
            }
            if (!octree) {
                sendError(socket, "Failed to compute octree for: " + objectName, idCode);
                return;
            }
            sampledRef = CCCoreLib::CloudSamplingTools::subsampleCloudWithOctreeAtLevel(
                cloud,
                static_cast<unsigned char>(level),
                CCCoreLib::CloudSamplingTools::NEAREST_POINT_TO_CELL_CENTER,
                nullptr,
                octree.data());
        } else {
            sendError(socket, "Unknown method: " + method + ". Use 'random', 'spatial' or 'octree'", idCode);
            return;
        }

        if (!sampledRef) {
            sendError(socket, "Subsampling failed on: " + objectName, idCode);
            return;
        }

        // 从 ReferenceCloud 生成新点云
        int warnings = 0;
        ccPointCloud* newCloud = cloud->partialClone(sampledRef, &warnings);
        delete sampledRef;
        sampledRef = nullptr;

        if (!newCloud) {
            sendError(socket, "Not enough memory to clone subsampled cloud", idCode);
            return;
        }

        if (warnings) {
            m_app->dispToConsole("[TcpPlugin][Subsample] Warning: colors/normals/SF may be missing", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
        }

        // 配置新点云属性
        const QString resultName = outputName.isEmpty() ? cloud->getName() + ".subsampled" : outputName;
        newCloud->setName(resultName);
        newCloud->copyGlobalShiftAndScale(*cloud);
        newCloud->setDisplay(cloud->getDisplay());
        newCloud->prepareDisplayForRefresh();

        if (cloud->getParent()) {
            cloud->getParent()->addChild(newCloud);
        }

        m_app->addToDB(newCloud);
        m_app->refreshAll();
        m_app->updateUI();

        m_app->dispToConsole(
            QString("[TcpPlugin][Subsample] '%1' -> '%2': %3 -> %4 points (%5)")
                .arg(cloud->getName())
                .arg(resultName)
                .arg(cloud->size())
                .arg(newCloud->size())
                .arg(method));

        QJsonObject result;
        result["inputCount"] = static_cast<int>(cloud->size());
        result["outputCount"] = static_cast<int>(newCloud->size());
        result["outputName"] = resultName;
        sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact), idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::merge(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QJsonArray nameArray = params["names"].toArray();
        const QString outputName = params["outputName"].toString();
        const bool addSourceSF = params["addSourceIndexSF"].toBool(false);

        if (nameArray.size() < 2) {
            sendError(socket, "At least 2 cloud names required in 'names'", idCode);
            return;
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        // ---- 1. 解析所有点云 ----
        // 辅助函数：按名称找到 ccPointCloud（直接匹配或取第一个 POINT_CLOUD 子节点）
        auto findCloud = [&](const QString& name) -> ccPointCloud* {
            ccHObject* found = findByName(root, name);
            if (!found) {
                return nullptr;
            }
            if (found->isA(CC_TYPES::POINT_CLOUD)) {
                return static_cast<ccPointCloud*>(found);
            }
            for (unsigned i = 0; i < found->getChildrenNumber(); ++i) {
                if (found->getChild(i)->isA(CC_TYPES::POINT_CLOUD)) {
                    return static_cast<ccPointCloud*>(found->getChild(i));
                }
            }
            return nullptr;
        };

        std::vector<ccPointCloud*> clouds;
        for (const QJsonValue& val : nameArray) {
            const QString name = val.toString();
            ccPointCloud* cloud = findCloud(name);
            if (!cloud) {
                sendError(socket, "Point cloud not found: " + name, idCode);
                return;
            }
            clouds.push_back(cloud);
        }

        // ---- 2. 预计算总点数 ----
        unsigned totalSize = 0;
        for (ccPointCloud* c : clouds) {
            totalSize += c->size();
        }

        // ---- 3. clone 第一个云作为合并目标（不破坏原始数据）----
        ccPointCloud* mergedCloud = clouds[0]->cloneThis(nullptr, true);
        if (!mergedCloud) {
            sendError(socket, "Not enough memory to clone base cloud", idCode);
            return;
        }

        // 预分配最终所需点数
        if (!mergedCloud->reserve(totalSize)) {
            delete mergedCloud;
            sendError(socket, "Not enough memory to reserve space for merged cloud", idCode);
            return;
        }

        // ---- 4. 可选：添加来源索引标量场 ----
        CCCoreLib::ScalarField* ocIndexSF = nullptr;
        if (addSourceSF) {
            int sfIdx = mergedCloud->getScalarFieldIndexByName(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
            if (sfIdx < 0) {
                sfIdx = mergedCloud->addScalarField(CC_ORIGINAL_CLOUD_INDEX_SF_NAME);
            }
            if (sfIdx < 0) {
                delete mergedCloud;
                sendError(socket, "Failed to allocate source-index scalar field", idCode);
                return;
            }
            ocIndexSF = mergedCloud->getScalarField(sfIdx);
            if (ocIndexSF) {
                ocIndexSF->fill(0); // 第一个云的点索引为 0
                mergedCloud->setCurrentDisplayedScalarField(sfIdx);
            }
        }

        // ---- 5. 依次 append 后续点云 ----
        for (size_t i = 1; i < clouds.size(); ++i) {
            ccPointCloud* pc = clouds[i];
            unsigned countBefore = mergedCloud->size();
            unsigned countAdded = pc->size();

            // append 不重算 SF min/max（最后统一算）
            mergedCloud->append(pc, countBefore, false, false);

            if (mergedCloud->size() != countBefore + countAdded) {
                delete mergedCloud;
                sendError(socket, QString("Merge failed at cloud '%1' (not enough memory?)").arg(pc->getName()), idCode);
                return;
            }

            if (ocIndexSF) {
                ScalarType index = static_cast<ScalarType>(i);
                for (unsigned j = 0; j < countAdded; ++j) {
                    ocIndexSF->setValue(countBefore + j, index);
                }
            }
        }

        // ---- 6. 统一计算所有 SF 的 min/max ----
        for (unsigned i = 0; i < mergedCloud->getNumberOfScalarFields(); ++i) {
            mergedCloud->getScalarField(i)->computeMinAndMax();
        }

        if (ocIndexSF) {
            mergedCloud->showSF(true);
        }

        // ---- 7. 设置名称、shift/scale，加入 DB ----
        const QString resultName = outputName.isEmpty()
                                   ? clouds[0]->getName() + "_merged"
                                   : outputName;
        mergedCloud->setName(resultName);
        mergedCloud->copyGlobalShiftAndScale(*clouds[0]);
        mergedCloud->setDisplay(clouds[0]->getDisplay());
        mergedCloud->prepareDisplayForRefresh();

        m_app->addToDB(mergedCloud);
        m_app->refreshAll();
        m_app->updateUI();

        m_app->dispToConsole(
            QString("[TcpPlugin][Merge] %1 clouds -> '%2': %3 points total")
                .arg(clouds.size())
                .arg(resultName)
                .arg(mergedCloud->size()));

        QJsonObject result;
        result["outputName"] = resultName;
        result["outputCount"] = static_cast<int>(mergedCloud->size());
        result["inputCount"] = static_cast<int>(clouds.size());
        sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact), idCode); },
	                          Qt::QueuedConnection);
}

void PointCloudService::clone(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        const QString objectName = params["name"].toString();
        const QString outputName = params["outputName"].toString();

        if (objectName.isEmpty()) {
            sendError(socket, "Missing 'name' parameter", idCode);
            return;
        }

        ccHObject* root = getDbRoot(socket, idCode);
        if (!root) {
            return;
        }

        ccHObject* entity = findByName(root, objectName);
        if (!entity) {
            sendError(socket, "Object not found: " + objectName, idCode);
            return;
        }

        // 克隆对象
        ccHObject* clone = nullptr;
        if (entity->isKindOf(CC_TYPES::POINT_CLOUD)) {
            clone = ccHObjectCaster::ToGenericPointCloud(entity)->clone();
        } else if (entity->isKindOf(CC_TYPES::PRIMITIVE)) {
            clone = static_cast<ccGenericPrimitive*>(entity)->clone();
        } else if (entity->isA(CC_TYPES::MESH)) {
            clone = ccHObjectCaster::ToMesh(entity)->cloneMesh();
        } else if (entity->isA(CC_TYPES::POLY_LINE)) {
            clone = ccHObjectCaster::ToPolyline(entity)->clone();
        } else {
            sendError(socket, "Unsupported entity type for cloning: " + objectName, idCode);
            return;
        }

        if (!clone) {
            sendError(socket, "Clone failed (not enough memory?): " + objectName, idCode);
            return;
        }

        // 设置名称、变换历史、显示
        const QString resultName = outputName.isEmpty() ? entity->getName() + "_clone" : outputName;
        clone->setName(resultName);
        clone->setGLTransformationHistory(entity->getGLTransformationHistory());
        clone->setDisplay(entity->getDisplay());

        m_app->addToDB(clone);
        m_app->updateUI();

        m_app->dispToConsole("[TcpPlugin][Clone] '" + objectName + "' -> '" + resultName + "'");

        QJsonObject result;
        result["outputName"] = resultName;
        sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact), idCode); },
	                          Qt::QueuedConnection);
}

bool PointCloudService::acquirePcdInternal(const QJsonObject& params,
                                           QTcpSocket*        socket,
                                           const QString&     idCode,
                                           QJsonObject*       result)
{
	struct SensorConfig
	{
		int                    deviceId         = 0;
		int                    xImageSize       = 3200;
		int                    maxLineSize      = 6400;
		int                    usePcImageFilter = 1;
		int                    timeout_ms       = 50000;
		LJS8IF_ETHERNET_CONFIG ethernet         = {{10, 10, 10, 234}, 24691};
		int                    highSpeedPortNo  = 24692;
	} cfg;

	const bool    useAsync    = params["async"].toBool(false);
	const QString outputName  = params["outputName"].toString("AcquiredCloud");
	const int     totalPixels = cfg.xImageSize * cfg.maxLineSize;

	if (static_cast<int>(m_heightBuf.size()) < totalPixels)
	{
		m_heightBuf.resize(totalPixels);
		m_luminanceBuf.resize(totalPixels);
	}

	std::fill(m_heightBuf.begin(), m_heightBuf.begin() + totalPixels, 0u);
	std::fill(m_luminanceBuf.begin(), m_luminanceBuf.begin() + totalPixels, 0u);

	unsigned short* pwHeightImage     = m_heightBuf.data();
	unsigned char*  pbyLuminanceImage = m_luminanceBuf.data();

	LJS8_ACQ_SETPARAM setParam{};
	setParam.timeout_ms         = cfg.timeout_ms;
	setParam.useExternalTrigger = 0;
	setParam.usePcImageFilter   = cfg.usePcImageFilter;

	LJS8_ACQ_GETPARAM getParam{};

	LJS8IF_Initialize();

	int errCode = LJS8_ACQ_OpenDevice(cfg.deviceId, &cfg.ethernet, cfg.highSpeedPortNo);
	if (errCode != LJS8IF_RC_OK)
	{
		LJS8IF_Finalize();
		sendError(socket, QString("Failed to open device (err=%1)").arg(errCode), idCode);
		return false;
	}

	if (!useAsync)
	{
		errCode = LJS8_ACQ_Acquire(cfg.deviceId, pwHeightImage, pbyLuminanceImage, &setParam, &getParam);
	}
	else
	{
		errCode = LJS8_ACQ_StartAsync(cfg.deviceId, &setParam);
		if (errCode == LJS8IF_RC_OK)
		{
			const DWORD start = timeGetTime();
			while (true)
			{
				if (timeGetTime() - start > static_cast<DWORD>(cfg.timeout_ms))
				{
					break;
				}
				errCode = LJS8_ACQ_AcquireAsync(cfg.deviceId, pwHeightImage, pbyLuminanceImage, &setParam, &getParam);
				if (errCode == LJS8IF_RC_OK)
				{
					break;
				}
			}
		}
	}

	LJS8_ACQ_CloseDevice(cfg.deviceId);
	LJS8IF_Finalize();

	if (errCode != LJS8IF_RC_OK)
	{
		sendError(socket, QString("Acquisition failed (err=%1)").arg(errCode), idCode);
		return false;
	}

	const int   xNum   = getParam.x_pointnum;
	const int   yNum   = getParam.y_linenum_acquired;
	const float xPitch = 12.5f / 1000.0f;
	const float yPitch = 12.5f / 1000.0f;
	const float zPitch = getParam.z_pitch_um / 1000.0f;

	unsigned validCount = 0;
	for (int i = 0; i < yNum * xNum; ++i)
	{
		if (m_heightBuf[i] != 0)
		{
			++validCount;
		}
	}

	if (validCount == 0)
	{
		sendError(socket, "Acquisition succeeded but all points are invalid", idCode);
		return false;
	}

	ccPointCloud* cloud = new ccPointCloud(outputName);
	if (!cloud->reserve(validCount))
	{
		delete cloud;
		sendError(socket, "Not enough memory to allocate point cloud", idCode);
		return false;
	}

	const unsigned short* ptr = m_heightBuf.data();
	for (int y = 0; y < yNum; ++y)
	{
		for (int x = 0; x < xNum; ++x, ++ptr)
		{
			if (*ptr == 0)
			{
				continue;
			}

			cloud->addPoint(CCVector3(
			    static_cast<PointCoordinateType>(x * xPitch),
			    static_cast<PointCoordinateType>(y * yPitch),
			    static_cast<PointCoordinateType>((*ptr - COLLECT_VALUE) * zPitch)));
		}
	}

	const bool    savePcd  = params["savePcd"].toBool(false);
	const QString savePath = params["savePath"].toString();

	if (savePcd)
	{
		if (savePath.isEmpty())
		{
			delete cloud;
			sendError(socket, "savePcd is true but savePath is empty", idCode);
			return false;
		}

		std::ofstream stream(savePath.toStdString());
		if (!stream)
		{
			delete cloud;
			sendError(socket, "Failed to open file for writing: " + savePath, idCode);
			return false;
		}

		stream << "# .PCD v0.7 - Point Cloud Data file format\n";
		stream << "VERSION 0.7\n";
		stream << "FIELDS x y z\n";
		stream << "SIZE 4 4 4\n";
		stream << "TYPE F F F\n";
		stream << "COUNT 1 1 1\n";
		stream << "WIDTH " << validCount << "\n";
		stream << "HEIGHT 1\n";
		stream << "VIEWPOINT 0 0 0 1 0 0 0\n";
		stream << "POINTS " << validCount << "\n";
		stream << "DATA ascii\n";

		char buf[64];
		ptr = m_heightBuf.data();
		for (int y = 0; y < yNum; ++y)
		{
			for (int x = 0; x < xNum; ++x, ++ptr)
			{
				if (*ptr == 0)
				{
					continue;
				}

				const float fx  = x * xPitch;
				const float fy  = y * yPitch;
				const float fz  = static_cast<float>((*ptr - COLLECT_VALUE) * zPitch);
				const int   len = snprintf(buf, sizeof(buf), "%.4f %.4f %.4f\n", fx, fy, fz);
				stream.write(buf, len);
			}
		}

		stream.close();
		m_app->dispToConsole("[TcpPlugin][AcquirePcd] PCD saved: " + savePath);
	}

	m_app->addToDB(cloud);
	m_app->refreshAll();
	m_app->updateUI();

	m_app->dispToConsole(
	    QString("[TcpPlugin][AcquirePcd] '%1': %2 valid points (%3x%4)")
	        .arg(outputName)
	        .arg(validCount)
	        .arg(xNum)
	        .arg(yNum));

	if (result)
	{
		(*result)["outputName"] = outputName;
		(*result)["pointCount"] = static_cast<int>(validCount);
		(*result)["xPoints"]    = xNum;
		(*result)["yLines"]     = yNum;
		if (savePcd)
		{
			(*result)["savedPath"] = savePath;
		}
	}

	return true;
}

void PointCloudService::acquirePcd(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QMetaObject::invokeMethod(qApp, [this, params, socket, idCode]()
	                          {
        QJsonObject result;
        if (acquirePcdInternal(params, socket, idCode, &result)) {
            sendOk(socket, QJsonDocument(result).toJson(QJsonDocument::Compact), idCode);
        } },
	                          Qt::QueuedConnection);
}

void PointCloudService::partInspectFunc(const QJsonObject& params)
{
    // 1. 从 params 中获取工件类型
    QString partType = params.value("partType").toString();
    
    // 2. 从 params 中获取 RFID
    QString rfid = params.value("rfid").toString();

    // 3. 找到对应的扫描配置 JSON 文件
    QString appDir = QCoreApplication::applicationDirPath();
    QString templateDir = appDir + "/PartInfo";
    QString configFile = templateDir + "/" + partType + "_inspect_config.json";

    QFile file(configFile);
    if (!file.exists()) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = QString("Configuration file not found for part type: %1").arg(partType);
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = QString("Failed to open configuration file: %1").arg(file.errorString());
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    // 3. 读取 JSON 文件
    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = QString("Invalid configuration file: %1").arg(parseError.errorString());
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    QJsonObject config = doc.object();
    QJsonArray holePositions = config.value("holePositions").toArray();
    if (holePositions.isEmpty()) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = "No hole positions found in configuration";
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    // 4. 生成并发送 NC 文件
    QString templateFile = templateDir + "/Inspect.nc";
    QFile templateNc(templateFile);
    if (!templateNc.exists()) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = "Inspect.nc template file does not exist";
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    if (!templateNc.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QJsonObject result;
        QJsonObject obj;
        obj["result"] = "failed";
        obj["error"] = "Failed to open Inspect.nc template";
        result["inspectResult"] = obj;
        savePartInspectResult(rfid, result);
        return;
    }

    const QString templateContent = QTextStream(&templateNc).readAll();
    templateNc.close();

    QString errorMessage;

    // 5. 处理每个打孔位置
    for (int i = 0; i < holePositions.size(); ++i) {
        QJsonObject holePos = holePositions[i].toObject();
        QJsonArray capturePositions = holePos.value("capturePositions").toArray();
        
        if (capturePositions.isEmpty()) {
            continue;
        }

        // 处理每个拍摄位置
        for (int j = 0; j < capturePositions.size(); ++j) {
            QJsonObject capturePos = capturePositions[j].toObject();
            double x = capturePos.value("X").toDouble();
            double y = capturePos.value("Y").toDouble();
            double z = capturePos.value("Z").toDouble();
            double b = capturePos.value("B").toDouble();
            double c = capturePos.value("C").toDouble();

            // 生成 NC 文件
            QString content = templateContent;
            content.replace("{X}", QString::number(x));
            content.replace("{Y}", QString::number(y));
            content.replace("{Z}", QString::number(z));
            content.replace("{B}", QString::number(b));
            content.replace("{C}", QString::number(c));

            const QString outputFile = templateDir + QString("/Inspect_%1_%2.nc").arg(i + 1).arg(j + 1);
            QFile outputNc(outputFile);
            if (!outputNc.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Failed to write NC file for position %1-%2").arg(i + 1).arg(j + 1);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }
            QTextStream out(&outputNc);
            out << content;
            outputNc.close();

            // 发送文件到机床
            if (!sendFileToMachine(outputFile, &errorMessage)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Failed to send NC file for position %1-%2: %3").arg(i + 1).arg(j + 1).arg(errorMessage);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }

            // 设置主程序
            if (!setMainProgram(&errorMessage)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Failed to set main program for position %1-%2: %3").arg(i + 1).arg(j + 1).arg(errorMessage);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }

            // 启动机床
            if (!startMachine(&errorMessage)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Failed to start machine for position %1-%2: %3").arg(i + 1).arg(j + 1).arg(errorMessage);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }

            // 等待机床完成
            if (!waitForMachineIdle(120, &errorMessage)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Machine did not become idle for position %1-%2: %3").arg(i + 1).arg(j + 1).arg(errorMessage);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }

            // 6. 获取点云
            const QString cloudName = QString("Hole_%1_Capture_%2").arg(i + 1).arg(j + 1);
            QJsonObject acquireParams;
            acquireParams["async"] = true;
            acquireParams["outputName"] = cloudName;
            if (!acquirePcdInternal(acquireParams, nullptr, QString(), nullptr)) {
                QJsonObject result;
                QJsonObject obj;
                obj["result"] = "failed";
                obj["error"] = QString("Failed to acquire point cloud for position %1-%2").arg(i + 1).arg(j + 1);
                result["inspectResult"] = obj;
                savePartInspectResult(rfid, result);
                return;
            }
        }
    }

    // 完成检查
    QJsonObject result;
    QJsonObject obj;
    obj["result"] = "success";
    obj["message"] = "Part inspection completed";
    result["inspectResult"] = obj;
    savePartInspectResult(rfid, result);
}

void PointCloudService::calibrationFunc(const QJsonObject& params)
{
	QString                  errorMessage;
	const QVector<QVector3D> positions = resolveCalibrationPositions(params, &errorMessage);
	if (positions.isEmpty())
	{

		m_Status = MachineStatus::Idle;
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = errorMessage.isEmpty() ? "No calibration positions provided" : errorMessage;
		m_calibrationResult["calibrationResult"] = obj;
		saveCalibrationStatus();
		return;
	}

	if (!waitForMachineIdle(1, &errorMessage))
	{
		m_Status = MachineStatus::Idle;
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = errorMessage.isEmpty() ? "Machine is not idle" : errorMessage;
		m_calibrationResult["calibrationResult"] = obj;
		saveCalibrationStatus();
		return;
	}

	QString mode;
	if (!getMachineMode(mode, &errorMessage))
	{
		m_Status = MachineStatus::Idle;
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = errorMessage.isEmpty() ? "Failed to get machine mode" : errorMessage;
		m_calibrationResult["calibrationResult"] = obj;
		saveCalibrationStatus();
		return;
	}
	if (mode != "Auto")
	{
		m_Status = MachineStatus::Idle;
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = QString("Machine mode must be Auto, current mode is '%1'").arg(mode);
		m_calibrationResult["calibrationResult"] = obj;
		saveCalibrationStatus();
		return;
	}

	if (!clearDbInternal(nullptr, ""))
	{
		m_Status = MachineStatus::Idle;
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = QString("Failed to clear DB before calibration");
		m_calibrationResult["calibrationResult"] = obj;
		saveCalibrationStatus();
		return;
	}

	std::vector<Eigen::Vector3d> machinePoints;
	std::vector<Eigen::Vector3d> scannerPoints;
	QJsonArray                   fitResults;
	machinePoints.reserve(static_cast<size_t>(positions.size()));
	scannerPoints.reserve(static_cast<size_t>(positions.size()));

	QString appDir       = QCoreApplication::applicationDirPath();
	QString templateDir  = appDir + "/Template";
	QString templateFile = templateDir + "/Calibration.nc";

	QDir dir(templateDir);
	if (!dir.exists())
	{
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = "Calibration template directory does not exist";
		m_calibrationResult["calibrationResult"] = obj;
		return;
	}

	QFile templateNc(templateFile);
	if (!templateNc.exists())
	{
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = "Calibration.nc template file does not exist";
		m_calibrationResult["calibrationResult"] = obj;
		return;
	}

	if (!templateNc.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = "Failed to open Calibration.nc template";
		m_calibrationResult["calibrationResult"] = obj;
		return;
	}
	const QString templateContent = QTextStream(&templateNc).readAll();
	templateNc.close();

	for (int i = 0; i < positions.size(); ++i)
	{
		const QVector3D& pos = positions[i];
		machinePoints.emplace_back(pos.x(), pos.y(), pos.z());

		QString content = templateContent;
		content.replace("{X}", QString::number(pos.x()));
		content.replace("{Y}", QString::number(pos.y()));
		content.replace("{Z}", QString::number(pos.z()));
		content.replace("{B}", QString::number(0));
		content.replace("{C}", QString::number(0));

		const QString outputFile = templateDir + QString("/Calibration_%1.nc").arg(i + 1);
		QFile         outputNc(outputFile);
		if (!outputNc.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QJsonObject obj;
			obj["result"]                            = "failed";
			obj["error"]                             = QString("Failed to write NC file for position %1").arg(i + 1);
			m_calibrationResult["calibrationResult"] = obj;
			return;
		}
		QTextStream out(&outputNc);
		out << content;
		outputNc.close();

		if (!sendFileToMachine(outputFile, &errorMessage))
		{
			QJsonObject obj;
			obj["result"]                            = "failed";
			obj["error"]                             = QString("Failed to send NC file for position %1: %2").arg(i + 1).arg(errorMessage);
			m_calibrationResult["calibrationResult"] = obj;
			return;
		}

		if (!setMainProgram(&errorMessage))
		{
			QJsonObject obj;
			obj["result"]                            = "failed";
			obj["error"]                             = QString("Failed to set main program for position %1: %2").arg(i + 1).arg(errorMessage);
			m_calibrationResult["calibrationResult"] = obj;
			return;
		}

		if (!startMachine(&errorMessage))
		{
			QJsonObject obj;
			obj["result"]                            = "failed";
			obj["error"]                             = QString("Failed to start machine for position %1: %2").arg(i + 1).arg(errorMessage);
			m_calibrationResult["calibrationResult"] = obj;
			return;
		}

		if (!waitForMachineIdle(120, &errorMessage))
		{
			QJsonObject obj;
			obj["result"]                            = "failed";
			obj["error"]                             = QString("Machine did not become idle for position %1: %2").arg(i + 1).arg(errorMessage);
			m_calibrationResult["calibrationResult"] = obj;
			return;
		}

		bool   fitSuccess = false;
		double centerX    = 0.0;
		double centerY    = 0.0;
		double centerZ    = 0.0;
		double rms        = 0.0;

		for (int retry = 0; retry < CALIBRATION_MAX_FIT_RETRIES && !fitSuccess; ++retry)
		{
			const QString cloudName = QString::number(i + 1);
			QJsonObject   acquireParams;
			acquireParams["async"]      = true;
			acquireParams["outputName"] = cloudName;
			if (!acquirePcdInternal(acquireParams, nullptr, QString(), nullptr))
			{
				continue;
			}

			QJsonObject fitParams;
			fitParams["type"]             = "sphere";
			fitParams["name"]             = cloudName;
			fitParams["outliersRatio"]    = 0.35;
			fitParams["confidence"]       = 0.9999;
			fitParams["autoDetectRadius"] = false;
			fitParams["radius"]           = CALIBRATION_RADIUS;
			fitParams["rms"]              = CALIBRATION_RMS_THRESHOLD;
			fitParams["retries"]          = 3;

			if (handleFitSphere(fitParams, nullptr, QString(), centerX, centerY, centerZ, rms) && rms < CALIBRATION_RMS_THRESHOLD)
			{
				fitSuccess = true;
				break;
			}

			ccHObject* root = m_app->dbRootObject();
			if (root)
			{
				if (ccHObject* target = findByName(root, cloudName))
				{
					m_app->removeFromDB(target);
				}
			}
		}

		if (!fitSuccess)
		{
			m_calibrationResult["result"] = "failed";
			m_calibrationResult["error"]  = QString("Sphere fitting failed at position %1 after %2 retries").arg(i + 1).arg(CALIBRATION_MAX_FIT_RETRIES);

			return;
		}

		scannerPoints.emplace_back(centerX, centerY, centerZ);

		QJsonObject fitItem;
		fitItem["index"]   = i + 1;
		fitItem["machine"] = QJsonArray{pos.x(), pos.y(), pos.z()};
		fitItem["scanner"] = QJsonArray{centerX, centerY, centerZ};
		fitItem["rms"]     = rms;
		fitResults.append(fitItem);
	}

	CalibrationRigidTransform transform;
	try
	{
		transform = computeRigidTransform(scannerPoints, machinePoints);
	}
	catch (const std::exception& e)
	{
		QJsonObject obj;
		obj["result"]                            = "failed";
		obj["error"]                             = QString("Calibration matrix computation failed: %1").arg(e.what());
		m_calibrationResult["calibrationResult"] = obj;
		return;
	}

	const Eigen::Matrix4d matrix = toMatrix4d(transform);
	QJsonArray            matrixRows;
	QJsonArray            residuals;
	bool                  residualOk = true;
	for (int row = 0; row < 4; ++row)
	{
		QJsonArray rowArray;
		for (int col = 0; col < 4; ++col)
		{
			rowArray.append(matrix(row, col));
		}
		matrixRows.append(rowArray);
	}

	for (size_t i = 0; i < scannerPoints.size(); ++i)
	{
		const Eigen::Vector3d calc  = transform.R * scannerPoints[i] + transform.T;
		const double          error = (calc - machinePoints[i]).norm();
		residuals.append(error);
		if (error > CALIBRATION_RESIDUAL_THRESHOLD)
		{
			residualOk = false;
		}
		m_app->dispToConsole(QString("[TcpPlugin][Calibration] Point %1 residual: %2 mm").arg(static_cast<int>(i + 1)).arg(error));
	}

	QString matrixText;
	for (int row = 0; row < 4; ++row)
	{
		matrixText += QString("%1 %2 %3 %4")
		                  .arg(matrix(row, 0))
		                  .arg(matrix(row, 1))
		                  .arg(matrix(row, 2))
		                  .arg(matrix(row, 3));
		if (row != 3)
		{
			matrixText += "\n";
		}
	}
	m_app->dispToConsole(QString("[TcpPlugin][Calibration]\n%1").arg(matrixText));

	// 保存标定结果
	QJsonObject obj;
	obj["result"] = residualOk ? "success" : "failed";
	if (residualOk)
	{
		obj["matrix"]            = matrixRows;
		obj["fitResults"]        = fitResults;
		obj["residuals"]         = residuals;
		obj["residualThreshold"] = CALIBRATION_RESIDUAL_THRESHOLD;
		obj["residualOk"]        = residualOk;
		obj["positionCount"]     = positions.size();
	}
	else
	{
		obj["error"] = QString("Calibration completed but residuals exceed threshold");
	}
	m_calibrationResult["calibrationResult"] = obj;
}

void PointCloudService::partInspect(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{

	QString strCmd = "PartInspect";

	// 1. 检查参数
	QString partType = params.value("partType").toString();
	if (partType.isEmpty()) {
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"] = "Part type is required";
		sendRes(socket, obj, idCode);
		return;
	}

	// 2. 检查 RFID
	QString rfid = params.value("rfid").toString();
	if (rfid.isEmpty()) {
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"] = "RFID is required";
		sendRes(socket, obj, idCode);
		return;
	}

	// 3. 先判断是否已经有标定结果，如果没有标定结果或者标定结果不是成功，则返回错误
	bool calibrationResult = false;
	do
	{
		if (!m_calibrationResult.contains("calibrationResult"))
		{
			break;
		}
		if (!m_calibrationResult["calibrationResult"].isObject())
		{
			break;
		}
		auto obj = m_calibrationResult["calibrationResult"].toObject();
		if (!obj.contains("result"))
		{
			break;
		}
		if (obj.contains("result") && obj["result"].isString() && obj["result"].toString() == "success")
		{
			calibrationResult = true;
		}
	} while (0);

	if (!calibrationResult)
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Calibration has not been successfully completed";
		sendRes(socket, obj, idCode);
		return;
	}

	// 4. 机床状态检测
	QString errorMessage;
	if (!waitForMachineIdle(1, &errorMessage))
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Machine is not idle";
		sendRes(socket, obj, idCode);
		return;
	}

	QString mode;
	if (!getMachineMode(mode, &errorMessage))
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Failed to get machine mode";
		sendRes(socket, obj, idCode);
		return;
	}
	if (mode != "Auto")
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = QString("Machine mode must be Auto, current mode is '%1'").arg(mode);	
		sendRes(socket, obj, idCode);
		return;
	}

	// 立即返回 OK
	QJsonObject obj;
	obj[strCmd + "_Ret"] = "0";
	sendRes(socket, obj, idCode);
	m_Status = MachineStatus::Running;

	QMetaObject::invokeMethod(qApp, [this, params]() {
		partInspectFunc(params);
		m_Status = MachineStatus::Idle;
	},
	Qt::QueuedConnection);
}

void PointCloudService::startCalibration(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QString strCmd = "Calibration";

	// 如果已经在标定中，直接返回错误
	if (m_Status == MachineStatus::Running)
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Calibration is already running";
		sendRes(socket, obj, idCode);
		return;
	}


	QString errorMessage;
	if (!waitForMachineIdle(1, &errorMessage))
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Machine is not idle";
		sendRes(socket, obj, idCode);
		return;
	}

	QString mode;
	if (!getMachineMode(mode, &errorMessage))
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = "Failed to get machine mode";
		sendRes(socket, obj, idCode);
		return;
	}
	if (mode != "Auto")
	{
		QJsonObject obj;
		obj[strCmd + "_Ret"] = "1";
		obj["Error"]         = QString("Machine mode must be Auto, current mode is '%1'").arg(mode);
		sendRes(socket, obj, idCode);
		return;
	}

	// 清空之前的标定结果
	m_calibrationResult["calibrationResult"] = 	QJsonObject{{"result", "calibrating"}, {"error", "machine is calibrating"}};
	m_Status                      = MachineStatus::Running;
	// 保存状态
	saveCalibrationStatus();

	// 立即返回 OK
	QJsonObject obj;
	obj[strCmd + "_Ret"] = "0";
	sendRes(socket, obj, idCode);

	QMetaObject::invokeMethod(qApp, [this, params]()
	                          {
		calibrationFunc(params);
		m_Status = MachineStatus::Idle;
		saveCalibrationStatus(); },
	                          Qt::QueuedConnection);
}

void PointCloudService::getStatus(const QJsonObject& params, QTcpSocket* socket, const QString& idCode)
{
	QJsonObject status;
	// 先获取机床状态，再获取标定状态
	QString value, errorMsg;
	if (!getDeviceRun(value, &errorMsg))
	{
		status["GetStatus_Ret"] = "1";
		status["Error"]         = "Failed to get device run status";
		sendRes(socket, status, idCode);
		return;
	}
	QByteArray responseBytes    = QJsonDocument(m_calibrationResult).toJson(QJsonDocument::Indented) + "\n";
	status["calibrationResult"] = m_calibrationResult["calibrationResult"];
	status["GetStatus_Ret"]     = "0";
	switch (m_Status)
	{
	case MachineStatus::Running:
		status["status"] = "running";
		break;
	case MachineStatus::Idle:
		if (value != "0")
		{
			status["status"] = "running";
		}
		else
		{
			status["status"] = "idle";
		}
		break;
	}
	sendRes(socket, status, idCode);
}
