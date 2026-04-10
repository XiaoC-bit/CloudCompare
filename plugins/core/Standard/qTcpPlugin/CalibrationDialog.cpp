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

const QVector<CalibrationDialog::Position> CalibrationDialog::DEFAULT_POSITIONS = {
    {0, 0, 0},
    {5, 0, 0},
    {5, 5, 0},
    {5, 10, 0},
    {10, 10, 0},
    {0, 0, 2.5},
    {0, 0, 5}
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
    
    // 开始标定流程
    bool calibrationSuccess = true;
    
    for (int i = 0; i < m_positions.size(); ++i) {
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
        if (!acquirePointCloud()) {
            calibrationSuccess = false;
            break;
        }
    }
    
    if (calibrationSuccess) {
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
    // 创建SendFile命令
    QJsonObject params;
	QString     strCmd = "SendFile";
	params["Command"]   = strCmd;
    params["Device"] = "CNC";
    params["LocalFile"] = filePath;
    
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

bool CalibrationDialog::startMachine()
{
    // 创建Start命令
    QJsonObject params;
	QString     strCmd = "Start";
	params["Command"]   = strCmd;
    params["Device"] = "CNC";
    
    QJsonObject response;
    if (!sendCommand(params, response, 10000)) {
        return false;
    }
    
    if (!response.contains(strCmd + "_Ret") || response[strCmd + "_Ret"].toString() != "0")
	{
        QMessageBox::critical(this, "错误", "启动机床失败: " + response[strCmd + "_message"].toString());
        return false;
    }
    
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
        QString     strCmd = "GetStatus";
        params["Command"]   = strCmd;
        params["Device"] = "CNC";
        
        QJsonObject response;
        if (!sendCommand(params, response, 5000)) {
            QThread::msleep(waitInterval);
            elapsedTime += waitInterval;
            continue;
        }
        
        if (response.contains("Status") && response["Status"].toString() == "IDLE") {
            return true;
        }
        
        QThread::msleep(waitInterval);
        elapsedTime += waitInterval;
    }
    
    QMessageBox::critical(this, "错误", "等待机床空闲超时");
    return false;
}

bool CalibrationDialog::acquirePointCloud()
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
		LJS8IF_ETHERNET_CONFIG ethernet         = {{192, 168, 0, 1}, 24691};
		int                    highSpeedPortNo  = 24692;
	} cfg;

	// ----------------------------------------------------------------
	// Runtime params from JSON
	// ----------------------------------------------------------------
	const bool    useAsync   = true;
	const QString outputName = "";

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
