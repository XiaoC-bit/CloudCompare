#include "CalibrationDialog.h"
#include "PointCloudService.h"
#include <Windows.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <ccMainAppInterface.h>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QThread>
#include <QUuid>
#include <ccPointCloud.h>

#include "LJS8_IF.h"
#include "LJS8_ErrorCode.h"
#include "LJS8_ACQ.h"

// 将 RigidTransform 转换为 4×4 齐次变换矩阵
Eigen::Matrix4d CalibrationDialog::toMatrix4d(const RigidTransform& tf)
{
	Eigen::Matrix4d T   = Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = tf.R;
	T.block<3, 1>(0, 3) = tf.T;
	return T;
}

// 输入：
//   scanner_points  扫描仪坐标系下的 N 个球心
//   machine_points  机床坐标系下对应的 N 个球心
// 输出：
//   RigidTransform {R, T}，满足 machine = R * scanner + T
CalibrationDialog::RigidTransform CalibrationDialog::computeRigidTransform(
    const std::vector<Eigen::Vector3d>& scanner_points,
    const std::vector<Eigen::Vector3d>& machine_points)
{
	const size_t N = scanner_points.size();
	if (N < 3 || N != machine_points.size())
		throw std::runtime_error("至少需要 3 个对应点");

	// 1. 计算质心
	Eigen::Vector3d c_scanner = Eigen::Vector3d::Zero();
	Eigen::Vector3d c_machine = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < N; ++i)
	{
		c_scanner += scanner_points[i];
		c_machine += machine_points[i];
	}
	c_scanner /= static_cast<double>(N);
	c_machine /= static_cast<double>(N);

	// 2. 去质心
	Eigen::MatrixXd Q_scanner(3, N), Q_machine(3, N);
	for (size_t i = 0; i < N; ++i)
	{
		Q_scanner.col(i) = scanner_points[i] - c_scanner;
		Q_machine.col(i) = machine_points[i] - c_machine;
	}

	// 3. 协方差矩阵 H = Q_scanner * Q_machine^T
	Eigen::Matrix3d H = Q_scanner * Q_machine.transpose();

	// 4. SVD 分解
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d                   U = svd.matrixU();
	Eigen::Matrix3d                   V = svd.matrixV();

	// 5. 计算旋转矩阵，修正镜像情况（det < 0）
	Eigen::Matrix3d R = V * U.transpose();
	if (R.determinant() < 0)
	{
		V.col(2) *= -1;
		R = V * U.transpose();
	}

	// 6. 计算平移向量
	Eigen::Vector3d T = c_machine - R * c_scanner;

	return {R, T};
}


const QVector<CalibrationDialog::Position> CalibrationDialog::DEFAULT_POSITIONS = {
    {0, 0, 0}
    /*,
    {5, 0, 0}
	,
    {5, 5, 0},
    {5, 10, 0},
    {10, 10, 0},
    {0, 0, 2.5},
    {0, 0, 5}*/
};

const QString CalibrationDialog::MACHINE_HOST = "localhost";

CalibrationDialog::CalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent) 
    :m_app(app),
	QDialog(parent)
    , m_socket(nullptr)
    , m_pointCloudService(pointCloudService)
    , m_positions(DEFAULT_POSITIONS)
{
    setWindowTitle("标定");
    setFixedSize(450, 400);
    setupUI();
    populateTable();
}

CalibrationDialog::~CalibrationDialog()
{
    if (m_socket) {
        m_socket->disconnectFromHost();
        delete m_socket;
    }
}

void CalibrationDialog::setupUI()
{
    m_mainLayout = new QVBoxLayout(this);
    
    m_tableWidget = new QTableWidget(this);
    m_tableWidget->setColumnCount(4);
    m_tableWidget->setHorizontalHeaderLabels({"X", "Y", "Z", "操作"});
    m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_tableWidget->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    m_tableWidget->setColumnWidth(3, 80);
    m_mainLayout->addWidget(m_tableWidget);
    
    m_addButton = new QPushButton("新增位置", this);
    connect(m_addButton, &QPushButton::clicked, this, &CalibrationDialog::onAddPosition);
    m_mainLayout->addWidget(m_addButton);
    
    m_buttonLayout = new QHBoxLayout();
    m_resetButton = new QPushButton("复位", this);
    connect(m_resetButton, &QPushButton::clicked, this, &CalibrationDialog::onReset);
    m_startButton = new QPushButton("开始标定", this);
    connect(m_startButton, &QPushButton::clicked, this, &CalibrationDialog::onStartCalibration);
    m_cancelButton = new QPushButton("取消", this);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    m_buttonLayout->addWidget(m_resetButton);
    m_buttonLayout->addWidget(m_startButton);
    m_buttonLayout->addWidget(m_cancelButton);
    m_mainLayout->addLayout(m_buttonLayout);
}

void CalibrationDialog::populateTable()
{
    m_tableWidget->setRowCount(m_positions.size());
    
    for (int i = 0; i < m_positions.size(); ++i) {
        const Position &pos = m_positions[i];
        
        QDoubleSpinBox *xSpinBox = new QDoubleSpinBox();
        xSpinBox->setValue(pos.x);
        xSpinBox->setMinimum(-1000);
        xSpinBox->setMaximum(1000);
        xSpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 0, xSpinBox);
        
        QDoubleSpinBox *ySpinBox = new QDoubleSpinBox();
        ySpinBox->setValue(pos.y);
        ySpinBox->setMinimum(-1000);
        ySpinBox->setMaximum(1000);
        ySpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 1, ySpinBox);
        
        QDoubleSpinBox *zSpinBox = new QDoubleSpinBox();
        zSpinBox->setValue(pos.z);
        zSpinBox->setMinimum(-1000);
        zSpinBox->setMaximum(1000);
        zSpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 2, zSpinBox);
        
        QPushButton *deleteButton = new QPushButton("删除");
        deleteButton->setProperty("row", i);
        connect(deleteButton, &QPushButton::clicked, this, &CalibrationDialog::onDeleteRow);
        m_tableWidget->setCellWidget(i, 3, deleteButton);
    }
}

void CalibrationDialog::onAddPosition()
{
    if (m_positions.size() >= MAX_POSITIONS) {
        QMessageBox::warning(this, "警告", "最多只能添加30组数据");
        return;
    }
    
    m_positions.append(Position(0, 0, 0));
    populateTable();
}

void CalibrationDialog::onStartCalibration()
{
    // 保存当前表格中的值到m_positions
    for (int i = 0; i < m_positions.size(); ++i) {
        QDoubleSpinBox *xSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 0));
        QDoubleSpinBox *ySpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 1));
        QDoubleSpinBox *zSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 2));
        
        if (xSpinBox && ySpinBox && zSpinBox) {
            m_positions[i].x = xSpinBox->value();
            m_positions[i].y = ySpinBox->value();
            m_positions[i].z = zSpinBox->value();
        }
    }
    
    // 检查机床状态是否空闲
    QJsonObject statusParams;
	/*
	{"Command":"GetDeviceRun","DeviceName":"CNC_1","DeviceType":"Cnc","Timeout":5}
	*/
	statusParams["Command"]    = "GetDeviceRun";
	statusParams["DeviceType"] = "Cnc";
	statusParams["DeviceName"] = "CNC_1";
	statusParams["Timeout"] = 5;
    QJsonObject statusResponse;
    if (!sendCommand(statusParams, statusResponse, 5000)) {
        return;
    }
    
    if (!statusResponse.contains("Value") || statusResponse["Value"].toString() != "0") {
        QMessageBox::warning(this, "警告", "当前测量机未处于Ready状态");
        return;
    }
    
    // 清空当前所有点云
    if (m_pointCloudService) {
        QJsonObject clearParams;
        // clearDB不需要参数
		// 信号槽 线程不同步 待修复
       // m_pointCloudService->clearDB(clearParams, nullptr, "");
    }

	std::vector<Eigen::Vector3d> machine_points, scanner_points;

    
    // 开始标定流程
    bool calibrationSuccess = true;
    
    for (int i = 0; i < m_positions.size(); ++i) {
		Eigen::Vector3d machine_point(m_positions[i].x, m_positions[i].y, m_positions[i].z);
		machine_points.push_back(machine_point);	

        const Position &pos = m_positions[i];
        
        // 1. 打开Template文件夹下的Calibration.nc文件
        QString appDir = QCoreApplication::applicationDirPath();
        QString templateDir = appDir + "/Template";
        QString templateFile = templateDir + "/Calibration.nc";
        QString outputFile = templateDir + "/Calibration_" + QString::number(i+1) + ".nc";
        
        // 检查Template文件夹是否存在
        QDir dir(templateDir);
        if (!dir.exists()) {
            QMessageBox::critical(this, "错误", "Template文件夹不存在");
            calibrationSuccess = false;
            break;
        }
        
        // 检查Calibration.nc文件是否存在
        QFile templateNc(templateFile);
        if (!templateNc.exists()) {
            QMessageBox::critical(this, "错误", "Calibration.nc文件不存在");
            calibrationSuccess = false;
            break;
        }
        
        // 2. 替换文件中的{X}, {Y}, {Z}
        if (!templateNc.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "错误", "无法打开Calibration.nc文件");
            calibrationSuccess = false;
            break;
        }
        
        QTextStream in(&templateNc);
        QString content = in.readAll();
        templateNc.close();
        
        content.replace("{X}", QString::number(pos.x));
        content.replace("{Y}", QString::number(pos.y));
        content.replace("{Z}", QString::number(pos.z));
        
        // 保存替换后的文件
        QFile outputNc(outputFile);
        if (!outputNc.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "错误", "无法保存替换后的NC文件");
            calibrationSuccess = false;
            break;
        }
        
        QTextStream out(&outputNc);
        out << content;
        outputNc.close();
        
        // 3. 发送NC文件到机床
        if (!sendFileToMachine(outputFile)) {
            calibrationSuccess = false;
            break;
        }

		//设置主程序
		if (!setMainProgram())
		{
			calibrationSuccess = false;
			break;
		}
        
        // 4. 发送启动机床命令
        if (!startMachine()) {
            calibrationSuccess = false;
            break;
        }
        
        // 5. 等待机床空闲
        if (!waitForMachineIdle()) {
            calibrationSuccess = false;
            break;
        }
        
        // 6. 获取点云数据
		if (!acquirePointCloud(QString::number(i + 1)))
		{
            calibrationSuccess = false;
            break;
        }
        
        // 7. 拟合球体
        if (m_pointCloudService) {
            QJsonObject fitParams;
            fitParams["type"] = "sphere";
			double          x, y, z, radius;

            // 假设最后获取的点云是当前要拟合的点云
			m_pointCloudService->handleFitSphere(fitParams, nullptr, "", x, y, z, radius);

			
			scanner_points.push_back(Eigen::Vector3d(x, y, z));
        }
    }


    
    if (calibrationSuccess) {

		//CalibrationDialog::RigidTransform  transform   = CalibrationDialog::computeRigidTransform(scanner_points, machine_points);
		//Eigen::Matrix4d T_cam2robot = CalibrationDialog::toMatrix4d(transform);


        QMessageBox::information(this, "成功", "标定完成");
        accept();
    } else {
        QMessageBox::warning(this, "失败", "标定过程中出现错误");
    }
}

QVector<QVector3D> CalibrationDialog::getDefaultPositions()
{
    QVector<QVector3D> positions;
    for (const Position &pos : DEFAULT_POSITIONS) {
        positions.append(QVector3D(pos.x, pos.y, pos.z));
    }
    return positions;
}

QVector<QVector3D> CalibrationDialog::getPositions() const
{
    QVector<QVector3D> positions;
    for (const Position &pos : m_positions) {
        positions.append(QVector3D(pos.x, pos.y, pos.z));
    }
    return positions;
}

void CalibrationDialog::onReset()
{
    m_positions = DEFAULT_POSITIONS;
    populateTable();
}

void CalibrationDialog::onDeleteRow()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) return;
    
    int row = button->property("row").toInt();
    
    if (m_positions.size() <= DEFAULT_POSITIONS.size()) {
        QMessageBox::warning(this, "警告", "数据组数不能少于默认值数量");
        return;
    }
    
    m_positions.removeAt(row);
    populateTable();
}

bool CalibrationDialog::connectToMachine()
{
    if (m_socket && m_socket->state() == QTcpSocket::ConnectedState) {
        return true;
    }
    
    if (m_socket) {
        delete m_socket;
    }
    
    m_socket = new QTcpSocket(this);
    m_socket->connectToHost(MACHINE_HOST, MACHINE_PORT);
    
    if (!m_socket->waitForConnected(5000)) {
        QMessageBox::critical(this, "错误", "无法连接到机床: " + m_socket->errorString());
        delete m_socket;
        m_socket = nullptr;
        return false;
    }
    
    return true;
}

bool CalibrationDialog::sendCommand(const QJsonObject &params, QJsonObject &response, int timeout)
{
    if (!connectToMachine()) {
        return false;
    }
    
    // 生成唯一ID
    QString idCode = QUuid::createUuid().toString();
    QJsonObject cmdParams = params;
    cmdParams["IDCode"] = idCode;
    
    // 发送命令
    QJsonDocument doc(cmdParams);
    QByteArray data = doc.toJson(QJsonDocument::Compact);
    
    if (m_socket->write(data) == -1) {
        QMessageBox::critical(this, "错误", "发送命令失败: " + m_socket->errorString());
        return false;
    }
    
    m_socket->flush();
    
    // 等待响应
    if (!m_socket->waitForReadyRead(timeout)) {
        QMessageBox::critical(this, "错误", "等待响应超时");
        return false;
    }
    
    // 读取响应
    QByteArray responseData = m_socket->readAll();
    QJsonParseError parseError;
    QJsonDocument responseDoc = QJsonDocument::fromJson(responseData, &parseError);
    
    if (parseError.error != QJsonParseError::NoError) {
        QMessageBox::critical(this, "错误", "解析响应失败: " + parseError.errorString());
        return false;
    }
    
    if (!responseDoc.isObject()) {
        QMessageBox::critical(this, "错误", "响应格式错误");
        return false;
    }
    
    response = responseDoc.object();
    return true;
}

bool CalibrationDialog::sendFileToMachine(const QString &filePath)
{

	/*

	{
	"CNCFile": "O1236",
	"CNCPath": "/c/",
	"Command": "SendFile",
	"DeviceName": "CNC_1",
	"DeviceType": "Cnc",
	"LocalFile": "D:\\SVN\\X-MASTER-PROJECT\\X-MASTER-8664-EDM-XIANHANGFA\\3 Source Code\\CloudCompare\\build\\qCC\\RelWithDebInfo\\Template\\Calibration_1.nc",
	"Timeout": 5
}
	*/
    // 创建SendFile命令
    QJsonObject params;
	QString     strCmd = "SendFile";
	params["Command"]   = strCmd;
	params["DeviceName"] = "CNC_1";
	params["DeviceType"] = "Cnc";
	params["CNCPath"] = "/c/";
	params["CNCFile"]    = "O1236";
	params["LocalFile"]  = filePath;
	params["Timeout"]  = 20;
    
    QJsonObject response;
    if (!sendCommand(params, response, 10000)) {
        return false;
    }
    
    if (!response.contains(strCmd + "_Ret") || response[strCmd + "_Ret"].toString() != "0")
	{
        QMessageBox::critical(this, "错误", "发送文件到机床失败: " + response["message"].toString());
        return false;
    }
    
    return true;
}


bool CalibrationDialog::setMainProgram(){
    /*
    {
    "Command": "SetMainProg",
    "DeviceName": "CNC_1",
    "DeviceType": "Cnc",
    "MainProg": "O1236",
    "Path": "/c/",
    "Timeout": 5
}
    */
        // 创建SetMainProg命令
    QJsonObject params;
    QString     strCmd = "SetMainProg";
    params["Command"]   = strCmd;
    params["DeviceType"] = "Cnc";
    params["DeviceName"] = "CNC_1";
    params["MainProg"] = "O1236";
    params["Path"] = "/c/";
    params["Timeout"] = 5;
        
    QJsonObject response;
    if (!sendCommand(params, response, 10000)) {
        return false;
    }
        
    if (!response.contains(strCmd + "_Ret") || response[strCmd + "_Ret"].toString() != "0")
	{
        QMessageBox::critical(this, "错误", "设置主程序失败: " + response[strCmd + "_message"].toString());
        return false;
    }
        
    return true;
}

bool CalibrationDialog::startMachine()
{
    // 创建Start命令
    QJsonObject params;
	/*
	{
	"Addr": "999",
	"AddrType": "R",
	"Bit": "0",
	"Command": "WritePlc",
	"DeviceName": "CNC_1",
	"DeviceType": "Cnc",
	"Timeout": 5,
	"Value": "0"
}
	*/
	QString     strCmd = "WritePlc";
	params["Command"]   = strCmd;
    params["DeviceType"] = "Cnc";
	params["DeviceName"] = "CNC_1";
    params["Addr"] = "999";
    params["AddrType"] = "R";
    params["Bit"] = "0";
    params["Timeout"] = 5;
    params["Value"] = "1";
    
    QJsonObject response;
    if (!sendCommand(params, response, 10000)) {
        return false;
    }
    
    if (!response.contains(strCmd + "_Ret") || response[strCmd + "_Ret"].toString() != "0")
	{
        QMessageBox::critical(this, "错误", "启动机床失败: " + response[strCmd + "_message"].toString());
        return false;
    }
    //休眠200ms
    QThread::msleep(200);

    //复位机床
    params["Value"] = "0";
     if (!sendCommand(params, response, 10000)) {
        return false;
    }
    
    if (!response.contains(strCmd + "_Ret") || response[strCmd + "_Ret"].toString() != "0")
	{
        QMessageBox::critical(this, "错误", "启动机床失败: " + response[strCmd + "_message"].toString());
        return false;
    }
    //休眠200ms
    QThread::msleep(200);
    
    return true;
}

bool CalibrationDialog::waitForMachineIdle()
{
    // 等待机床空闲，最多等待60秒
    int maxWaitTime = 60000; // 60秒
    int waitInterval = 1000; // 1秒
    int elapsedTime = 0;    
    while (elapsedTime < maxWaitTime) {
        // 创建GetStatus命令
        QJsonObject params;

        /*
        
	{"Command":"GetDeviceRun","DeviceName":"CNC_1","DeviceType":"Cnc","Timeout":5}
        */
        QString     strCmd = "GetDeviceRun";
        params["Command"]   = strCmd;
        params["DeviceName"] = "CNC_1";
        params["DeviceType"] = "Cnc";
        params["Timeout"] = 5;
        
        QJsonObject response;
        if (!sendCommand(params, response, 5000)) {
            QThread::msleep(waitInterval);
            elapsedTime += waitInterval;
            continue;
        }
        
        if (response.contains("Value") && response["Value"].toString() == "0") {
            return true;
        }
        
        QThread::msleep(waitInterval);
        elapsedTime += waitInterval;
    }
    
    QMessageBox::critical(this, "错误", "等待机床空闲超时");
    return false;
}

bool CalibrationDialog::acquirePointCloud(const QString& outputName)
{
	// ----------------------------------------------------------------
	// Hardware config (hardcoded — change to match your sensor)
	// ----------------------------------------------------------------
	struct SensorConfig
	{
		int                    deviceId         = 0;
		int                    xImageSize       = 3200;
		int                    maxLineSize      = 6400;
		int                    usePcImageFilter = 1;
		int                    timeout_ms       = 20000;
		LJS8IF_ETHERNET_CONFIG ethernet         = {{10, 10, 10, 234}, 24691};
		int                    highSpeedPortNo  = 24692;
	} cfg;

	// ----------------------------------------------------------------
	// Runtime params from JSON
	// ----------------------------------------------------------------
	const bool    useAsync   = true;

	// ----------------------------------------------------------------
	// Allocate buffers
	// ----------------------------------------------------------------
	const int totalPixels = cfg.xImageSize * cfg.maxLineSize;

	if (static_cast<int>(m_heightBuf.size()) < totalPixels)
	{
		m_heightBuf.resize(totalPixels);
		m_luminanceBuf.resize(totalPixels);
	}

	// 每次采集前清零
	std::fill(m_heightBuf.begin(), m_heightBuf.begin() + totalPixels, 0u);
	std::fill(m_luminanceBuf.begin(), m_luminanceBuf.begin() + totalPixels, 0u);

	unsigned short* pwHeightImage     = m_heightBuf.data();
	unsigned char*  pbyLuminanceImage = m_luminanceBuf.data();

	// ----------------------------------------------------------------
	// Prepare acquisition params
	// ----------------------------------------------------------------
	LJS8_ACQ_SETPARAM setParam{};
	setParam.timeout_ms         = cfg.timeout_ms;
	setParam.useExternalTrigger = 0;
	setParam.usePcImageFilter   = cfg.usePcImageFilter;

	LJS8_ACQ_GETPARAM getParam{};

	LJS8IF_Initialize();

	// ----------------------------------------------------------------
	// Step 1: Open device
	// ----------------------------------------------------------------
	int errCode = LJS8_ACQ_OpenDevice(cfg.deviceId, &cfg.ethernet, cfg.highSpeedPortNo);
	if (errCode != LJS8IF_RC_OK)
	{
		LJS8IF_Finalize();
		return false;
	}

	// ----------------------------------------------------------------
	// Step 2: Acquire (sync or async)
	// ----------------------------------------------------------------
	if (!useAsync)
	{
		// Synchronous — blocks until done or timeout
		errCode = LJS8_ACQ_Acquire(cfg.deviceId, pwHeightImage, pbyLuminanceImage, &setParam, &getParam);
	}
	else
	{
		// Asynchronous — start, then poll
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

	// ----------------------------------------------------------------
	// Step 3: Close device (always)
	// ----------------------------------------------------------------
	LJS8_ACQ_CloseDevice(cfg.deviceId);
	LJS8IF_Finalize();

	if (errCode != LJS8IF_RC_OK)
	{
		return false;
	}

	// ----------------------------------------------------------------
	// Step 4: Convert height image -> ccPointCloud
	// ----------------------------------------------------------------
	const int   xNum   = getParam.x_pointnum;
	const int   yNum   = getParam.y_linenum_acquired;
	const float xPitch = 12.5f / 1000.0f; // µm -> mm
	const float yPitch = 12.5f / 1000.0f;
	const float zPitch = getParam.z_pitch_um / 1000.0f;

	// Count valid points (0 = invalid)
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
		return false;
	}

	ccPointCloud* cloud = new ccPointCloud(outputName);
	if (!cloud->reserve(validCount))
	{
		delete cloud;
		return false;
	}

	static int    COLLECT_VALUE = 32768;
	static double INVALID_VALUE = -999.9999;

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



	// ----------------------------------------------------------------
	// Step 6: Add to DB
	// ----------------------------------------------------------------
	m_app->addToDB(cloud);
	m_app->refreshAll();
	m_app->updateUI();

	m_app->dispToConsole(
	    QString("[TcpPlugin][AcquirePcd] '%1': %2 valid points (%3x%4)")
	        .arg(outputName)
	        .arg(validCount)
	        .arg(xNum)
	        .arg(yNum));
	return true;
}
