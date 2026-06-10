#include "AcquirePcdDialog.h"
#include <QMessageBox>
#include <QThread>
#include <qeventloop.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccMainAppInterface.h>
#include <ccHObject.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QCoreApplication>
#include <QDir>


#include "PointCloudService.h"
#include "LJS8_IF.h"
#include "LJS8_ErrorCode.h"
#include "LJS8_ACQ.h"

AcquirePcdDialog::AcquirePcdDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : QDialog(parent)
    , m_app(app)
    , m_pointCloudService(pointCloudService)
    , m_isAcquiring(false)
    , m_cancelRequested(false)
{
    setWindowTitle("获取点云");
    setModal(true);
    setupUI();
    loadConfig();
}

AcquirePcdDialog::~AcquirePcdDialog()
{
    if (m_progressTimer) {
        m_progressTimer->stop();
        delete m_progressTimer;
    }
}

void AcquirePcdDialog::setupUI()
{
    setFixedSize(900, 900);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 16, 20, 16);
    mainLayout->setSpacing(10);

    // 信息卡片（示意图 + 操作提示）
    m_infoFrame = new QWidget(this);
    m_infoFrame->setMinimumHeight(170);
    m_infoFrame->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    m_infoFrame->setStyleSheet("QWidget { background-color: #f5f7fa; border: 1px solid #d0d7e0; border-radius: 8px; }");
    QHBoxLayout* infoLayout = new QHBoxLayout(m_infoFrame);
    infoLayout->setContentsMargins(16, 16, 16, 16);
    infoLayout->setSpacing(20);

    m_schematicLabel = new QLabel(this);
    m_schematicLabel->setFixedSize(560, 460);
    m_schematicLabel->setStyleSheet("QLabel { background-color: white; border: 2px solid #c0c8d0; border-radius: 6px; }");
    m_schematicLabel->setAlignment(Qt::AlignCenter);
    m_schematicLabel->setScaledContents(true);

    QPixmap schematicPixmap(":/CC/plugin/qTcpPlugin/res/camera_op.png");
    if (!schematicPixmap.isNull()) {
		m_schematicLabel->setPixmap(schematicPixmap.scaled(560, 460, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    } else {
        m_schematicLabel->setText(
            "<div style='text-align: center; color: #999;'>"
            "<div style='font-size: 14px;'>示意图</div>"
            "</div>"
        );
    }

    infoLayout->addWidget(m_schematicLabel);

    m_instructionLabel = new QLabel(this);
    m_instructionLabel->setWordWrap(true);
    m_instructionLabel->setStyleSheet("QLabel { color: #333; line-height: 1.6; }");
    m_instructionLabel->setText(
        "<h3 style='margin: 0 0 8px 0; color: #4a90d9;'>操作说明</h3>"
        "<p style='margin: 4px 0;'>1. 将待扫描物体（工件/电极）放置于<strong>机床工作台</strong>上</p>"
        "<p style='margin: 4px 0;'>2. 调节移动<strong>3D扫描仪</strong>至合适位置</p>"
        "<p style='margin: 4px 0;'>3. 调整成像距离，确保<strong>成像清晰</strong></p>"
        "<p style='margin: 10px 0 0 0; color: #cc6600; font-weight: bold;'>⚠️ 确认以上步骤完成后，再点击「开始获取」</p>"
    );

    infoLayout->addWidget(m_instructionLabel, 1);

    mainLayout->addWidget(m_infoFrame);

    // 拍摄选项
    QGroupBox* captureOptionGroup = new QGroupBox("拍摄选项", this);
    QVBoxLayout* captureOptionLayout = new QVBoxLayout(captureOptionGroup);
    captureOptionLayout->setSpacing(8);

    m_directCaptureBtn = new QRadioButton("直接采集点云", this);
    m_directCaptureBtn->setChecked(true);
    captureOptionLayout->addWidget(m_directCaptureBtn);

    m_alignCaptureBtn = new QRadioButton("采集后对齐到模型坐标系", this);
    captureOptionLayout->addWidget(m_alignCaptureBtn);

    mainLayout->addWidget(captureOptionGroup);

    // 当前拍摄点位
    m_capturePositionGroup = new QGroupBox("当前拍摄点位", this);
    QGridLayout* captureLayout = new QGridLayout(m_capturePositionGroup);
    captureLayout->setSpacing(10);

    captureLayout->addWidget(new QLabel("X:"), 0, 0);
    m_captureXEdit = new QLineEdit(this);
    m_captureXEdit->setFixedWidth(80);
    captureLayout->addWidget(m_captureXEdit, 0, 1);

    captureLayout->addWidget(new QLabel("Y:"), 0, 2);
    m_captureYEdit = new QLineEdit(this);
    m_captureYEdit->setFixedWidth(80);
    captureLayout->addWidget(m_captureYEdit, 0, 3);

    captureLayout->addWidget(new QLabel("Z:"), 0, 4);
    m_captureZEdit = new QLineEdit(this);
    m_captureZEdit->setFixedWidth(80);
    captureLayout->addWidget(m_captureZEdit, 0, 5);

    captureLayout->addWidget(new QLabel("B:"), 1, 0);
    m_captureBEdit = new QLineEdit(this);
    m_captureBEdit->setFixedWidth(80);
    captureLayout->addWidget(m_captureBEdit, 1, 1);

    captureLayout->addWidget(new QLabel("C:"), 1, 2);
    m_captureCEdit = new QLineEdit(this);
    m_captureCEdit->setFixedWidth(80);
    captureLayout->addWidget(m_captureCEdit, 1, 3);

    m_getCapturePosBtn = new QPushButton("从设备获取", this);
    m_getCapturePosBtn->setFixedSize(100, 28);
    captureLayout->addWidget(m_getCapturePosBtn, 1, 4, 1, 2);

    m_capturePositionGroup->setEnabled(false);
    mainLayout->addWidget(m_capturePositionGroup);

    // 模型原点的机械坐标
    m_modelOriginGroup = new QGroupBox("模型原点的机械坐标", this);
    QGridLayout* modelLayout = new QGridLayout(m_modelOriginGroup);
    modelLayout->setSpacing(10);

    modelLayout->addWidget(new QLabel("X:"), 0, 0);
    m_modelXEdit = new QLineEdit(this);
    m_modelXEdit->setFixedWidth(80);
    modelLayout->addWidget(m_modelXEdit, 0, 1);

    modelLayout->addWidget(new QLabel("Y:"), 0, 2);
    m_modelYEdit = new QLineEdit(this);
    m_modelYEdit->setFixedWidth(80);
    modelLayout->addWidget(m_modelYEdit, 0, 3);

    modelLayout->addWidget(new QLabel("Z:"), 0, 4);
    m_modelZEdit = new QLineEdit(this);
    m_modelZEdit->setFixedWidth(80);
    modelLayout->addWidget(m_modelZEdit, 0, 5);

    modelLayout->addWidget(new QLabel("B:"), 1, 0);
    m_modelBEdit = new QLineEdit(this);
    m_modelBEdit->setFixedWidth(80);
    modelLayout->addWidget(m_modelBEdit, 1, 1);

    modelLayout->addWidget(new QLabel("C:"), 1, 2);
    m_modelCEdit = new QLineEdit(this);
    m_modelCEdit->setFixedWidth(80);
    modelLayout->addWidget(m_modelCEdit, 1, 3);

    m_getModelOriginBtn = new QPushButton("从设备获取", this);
    m_getModelOriginBtn->setFixedSize(100, 28);
    modelLayout->addWidget(m_getModelOriginBtn, 1, 4, 1, 2);

    m_modelOriginGroup->setEnabled(false);
    mainLayout->addWidget(m_modelOriginGroup);

    // 状态标签
    m_statusLabel = new QLabel("准备获取点云", this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(m_statusLabel);

    // 进度条
    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 100);
    m_progressBar->setValue(0);
    m_progressBar->setVisible(false);
    mainLayout->addWidget(m_progressBar);

    // 按钮
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);

    m_saveConfigBtn = new QPushButton("保存配置", this);
    m_saveConfigBtn->setFixedSize(100, 35);
    buttonLayout->addWidget(m_saveConfigBtn);

    buttonLayout->addStretch();

    m_acquireBtn = new QPushButton("开始获取", this);
    m_acquireBtn->setFixedSize(120, 35);
    buttonLayout->addWidget(m_acquireBtn);

    m_cancelBtn = new QPushButton("取消", this);
    m_cancelBtn->setFixedSize(120, 35);
    m_cancelBtn->setEnabled(false);
    buttonLayout->addWidget(m_cancelBtn);

    buttonLayout->addStretch();
    mainLayout->addLayout(buttonLayout);

    m_progressTimer = new QTimer(this);

    connect(m_acquireBtn, &QPushButton::clicked, this, &AcquirePcdDialog::onAcquireClicked);
    connect(m_cancelBtn, &QPushButton::clicked, this, &AcquirePcdDialog::onCancelClicked);
    connect(m_progressTimer, &QTimer::timeout, this, &AcquirePcdDialog::onAcquisitionProgress);
    connect(m_directCaptureBtn, &QRadioButton::toggled, this, &AcquirePcdDialog::onRadioButtonChanged);
    connect(m_alignCaptureBtn, &QRadioButton::toggled, this, &AcquirePcdDialog::onRadioButtonChanged);
    connect(m_getCapturePosBtn, &QPushButton::clicked, this, &AcquirePcdDialog::onGetCapturePositionFromDevice);
    connect(m_getModelOriginBtn, &QPushButton::clicked, this, &AcquirePcdDialog::onGetModelOriginFromDevice);
    connect(m_saveConfigBtn, &QPushButton::clicked, this, &AcquirePcdDialog::onSaveConfig);
}

void AcquirePcdDialog::onAcquireClicked()
{
    m_isAcquiring = true;
    m_cancelRequested = false;
    m_acquireBtn->setEnabled(false);
    m_cancelBtn->setEnabled(true);
    m_progressBar->setVisible(true);
    m_progressBar->setValue(0);
    m_statusLabel->setText("正在初始化设备...");

    m_progressTimer->start(100);


	bool    success = acquirePointCloud("AcquiredCloud");
	QString errorMessage;

	if (!success && !m_cancelRequested)
	{
		errorMessage = "点云获取失败";
	}

	onAcquisitionFinished(success, errorMessage);

    /*QThread::create([this]() {
        bool success = acquirePointCloud("AcquiredCloud");
        QString errorMessage;

        if (!success && !m_cancelRequested) {
            errorMessage = "点云获取失败";
        }

        QMetaObject::invokeMethod(this, "onAcquisitionFinished",
            Qt::QueuedConnection,
            Q_ARG(bool, success),
            Q_ARG(QString, errorMessage));
    })->start();*/
}

void AcquirePcdDialog::onCancelClicked()
{
    m_cancelRequested = true;
    m_statusLabel->setText("正在取消...");
    m_cancelBtn->setEnabled(false);
}

void AcquirePcdDialog::onRadioButtonChanged()
{
    bool alignMode = m_alignCaptureBtn->isChecked();
    m_capturePositionGroup->setEnabled(alignMode);
    m_modelOriginGroup->setEnabled(alignMode);
}

void AcquirePcdDialog::onGetCapturePositionFromDevice()
{
    if (!m_pointCloudService) {
        QMessageBox::warning(this, "错误", "无法连接到点云服务");
        return;
    }

    double x, y, z, a, b, c;
    QString errorMessage;

    if (m_pointCloudService->getDeviceMainAxisCoor(x, y, z, a, b, c, &errorMessage)) {
        m_captureXEdit->setText(QString::number(x, 'f', 3));
        m_captureYEdit->setText(QString::number(y, 'f', 3));
        m_captureZEdit->setText(QString::number(z, 'f', 3));
        m_captureBEdit->setText(QString::number(a, 'f', 3));
        m_captureCEdit->setText(QString::number(b, 'f', 3));
    } else {
        QMessageBox::warning(this, "失败", QString("获取设备坐标失败: %1").arg(errorMessage));
    }
}

void AcquirePcdDialog::onGetModelOriginFromDevice()
{
    if (!m_pointCloudService) {
        QMessageBox::warning(this, "错误", "无法连接到点云服务");
        return;
    }

    double x, y, z, a, b, c;
    QString errorMessage;

    if (m_pointCloudService->getDeviceMainAxisCoor(x, y, z, a, b, c, &errorMessage)) {
        m_modelXEdit->setText(QString::number(x, 'f', 3));
        m_modelYEdit->setText(QString::number(y, 'f', 3));
        m_modelZEdit->setText(QString::number(z, 'f', 3));
        m_modelBEdit->setText(QString::number(a, 'f', 3));
        m_modelCEdit->setText(QString::number(b, 'f', 3));
    } else {
        QMessageBox::warning(this, "失败", QString("获取设备坐标失败: %1").arg(errorMessage));
    }
}

void AcquirePcdDialog::onSaveConfig()
{
    saveConfigToFile();
    QMessageBox::information(this, "成功", "配置已保存");
}

void AcquirePcdDialog::loadConfig()
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString configFile = appDir + "/config/AcquirePcdConfig.json";

    QFile file(configFile);
    if (!file.exists()) {
        return;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }

    QJsonObject obj = doc.object();

    if (obj.contains("capturePosition")) {
        QJsonObject pos = obj["capturePosition"].toObject();
        if (pos.contains("X")) m_captureXEdit->setText(pos["X"].toString());
        if (pos.contains("Y")) m_captureYEdit->setText(pos["Y"].toString());
        if (pos.contains("Z")) m_captureZEdit->setText(pos["Z"].toString());
        if (pos.contains("B")) m_captureBEdit->setText(pos["B"].toString());
        if (pos.contains("C")) m_captureCEdit->setText(pos["C"].toString());
    }

    if (obj.contains("modelOrigin")) {
        QJsonObject origin = obj["modelOrigin"].toObject();
        if (origin.contains("X")) m_modelXEdit->setText(origin["X"].toString());
        if (origin.contains("Y")) m_modelYEdit->setText(origin["Y"].toString());
        if (origin.contains("Z")) m_modelZEdit->setText(origin["Z"].toString());
        if (origin.contains("B")) m_modelBEdit->setText(origin["B"].toString());
        if (origin.contains("C")) m_modelCEdit->setText(origin["C"].toString());
    }

    if (obj.contains("alignMode")) {
        bool alignMode = obj["alignMode"].toBool();
        m_alignCaptureBtn->setChecked(alignMode);
        m_directCaptureBtn->setChecked(!alignMode);
        m_capturePositionGroup->setEnabled(alignMode);
        m_modelOriginGroup->setEnabled(alignMode);
    }
}

void AcquirePcdDialog::saveConfigToFile()
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString configFile = appDir + "/config/AcquirePcdConfig.json";

    QJsonObject obj;

    QJsonObject capturePos;
    capturePos["X"] = m_captureXEdit->text();
    capturePos["Y"] = m_captureYEdit->text();
    capturePos["Z"] = m_captureZEdit->text();
    capturePos["B"] = m_captureBEdit->text();
    capturePos["C"] = m_captureCEdit->text();
    obj["capturePosition"] = capturePos;

    QJsonObject modelOrigin;
    modelOrigin["X"] = m_modelXEdit->text();
    modelOrigin["Y"] = m_modelYEdit->text();
    modelOrigin["Z"] = m_modelZEdit->text();
    modelOrigin["B"] = m_modelBEdit->text();
    modelOrigin["C"] = m_modelCEdit->text();
    obj["modelOrigin"] = modelOrigin;

    obj["alignMode"] = m_alignCaptureBtn->isChecked();

    QJsonDocument doc(obj);
    QFile file(configFile);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        file.write(doc.toJson(QJsonDocument::Indented));
        file.close();
    }
}

void AcquirePcdDialog::onAcquisitionProgress()
{
    if (!m_isAcquiring) {
        return;
    }

    int currentValue = m_progressBar->value();
	if (currentValue < 30)
	{
		m_statusLabel->setText("正在初始化设备...");
        m_progressBar->setValue(currentValue + 5);
    }
	else if (currentValue < 90)
	{
		m_statusLabel->setText("正在扫描轮廓...");
		m_progressBar->setValue(currentValue + 1);
	}
}

void AcquirePcdDialog::onAcquisitionFinished(bool success, const QString& errorMessage)
{
    m_isAcquiring = false;
    m_progressTimer->stop();
    m_progressBar->setVisible(false);
    m_cancelBtn->setEnabled(false);
    m_acquireBtn->setEnabled(true);

    if (success) {
        m_statusLabel->setText("点云获取成功");
        //accept();
    } else {
        if (m_cancelRequested) {
            m_statusLabel->setText("已取消");
        } else {
            m_statusLabel->setText("点云获取失败");
            QMessageBox::warning(this, "失败", errorMessage);
        }
    }
}

bool AcquirePcdDialog::acquirePointCloud(const QString& outputName)
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

	const int totalPixels = cfg.xImageSize * cfg.maxLineSize;

	const bool                  flipX = true; // 控制X方向是否翻转
	std::vector<unsigned short> heightBuf(totalPixels, 0);
	std::vector<unsigned char>  luminanceBuf(totalPixels, 0);

	unsigned short* pwHeightImage     = heightBuf.data();
	unsigned char*  pbyLuminanceImage = luminanceBuf.data();

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
		return false;
	}

	errCode = LJS8_ACQ_StartAsync(cfg.deviceId, &setParam);
	if (errCode == LJS8IF_RC_OK)
	{
		const DWORD start = timeGetTime();
		while (true)
		{
			if (m_cancelRequested)
			{
				LJS8_ACQ_CloseDevice(cfg.deviceId);
				LJS8IF_Finalize();
				return false;
			}

			if (timeGetTime() - start > static_cast<DWORD>(cfg.timeout_ms))
			{
				break;
			}

			errCode = LJS8_ACQ_AcquireAsync(cfg.deviceId, pwHeightImage, pbyLuminanceImage, &setParam, &getParam);
			if (errCode == LJS8IF_RC_OK)
			{
				break;
			}

			QEventLoop loop;
			QTimer::singleShot(50, &loop, &QEventLoop::quit);
			loop.exec();
		}
	}

	LJS8_ACQ_CloseDevice(cfg.deviceId);
	LJS8IF_Finalize();

	if (errCode != LJS8IF_RC_OK)
	{
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
		if (heightBuf[i] != 0)
		{
			++validCount;
		}
	}

	if (validCount == 0)
	{
		return false;
	}

	ccPointCloud* cloud = new ccPointCloud(outputName);
	cloud->reserve(validCount);

	for (int y = 0; y < yNum; ++y)
	{
		for (int x = 0; x < xNum; ++x)
		{
			const int idx = y * xNum + x;

			const float fx = flipX ? (-x * xPitch) : (x * xPitch);
			if (heightBuf[idx] != 0)
			{
				CCVector3 P(
				    static_cast<PointCoordinateType>(fx),
			    static_cast<PointCoordinateType>(y * yPitch),
			    static_cast<PointCoordinateType>((heightBuf[idx] - 32768) * zPitch));
				cloud->addPoint(P);
			}
		}
	}

	bool                 lockedVertices = false;
	ccGenericPointCloud* pointCloud          = ccHObjectCaster::ToGenericPointCloud(cloud, &lockedVertices);
	if (pointCloud && lockedVertices)
	{
		return false;
	}
	if (pointCloud)
	{
		pointCloud->deleteOctree();
	}

	if (m_alignCaptureBtn->isChecked())
	{
		// 读取拍摄点位的XYZ和BC角度
		const double captureX   = m_captureXEdit->text().toDouble();
		const double captureY   = m_captureYEdit->text().toDouble();
		const double captureZ   = m_captureZEdit->text().toDouble();
		const double captureB   = m_captureBEdit->text().toDouble(); // 单位：度
		const double captureC   = m_captureCEdit->text().toDouble(); // 单位：度

		// 读取模型原点的机械坐标（B=0,C=0时工件原点对应的机床坐标）
		const double modelX     = m_modelXEdit->text().toDouble();
		const double modelY     = m_modelYEdit->text().toDouble();
		const double modelZ     = m_modelZEdit->text().toDouble();

		// 获取BC轴旋转中心（机床坐标系）
		const Eigen::Vector3d pivotB = m_pointCloudService->getBAxisCenter();
		const Eigen::Vector3d pivotC = m_pointCloudService->getCAxisCenter();

		// Step 1: 手眼矩阵 —— 将点云从相机坐标系变换到机床坐标系（在当前姿态下）
		// Step 2: 通过机床运动变换矩阵，将点云从当前机床姿态还原到B=0,C=0时的位置
		//         buildRobotMotion 构建的是从(0,0,0,0°,0°)到(captureX,Y,Z,B,C)的正向变换，
		//         取逆即为从拍摄姿态还原到零位。
		const Eigen::Matrix4d mRobotMotion = PointCloudService::buildRobotMotion(
		    -captureX, -captureY, -captureZ, captureB, captureC, pivotB, pivotC);

		// Step 3: 从机床零位平移到工件坐标系（模型原点即工件坐标系原点对应的机床坐标取反）
		Eigen::Matrix4d mMoveToPartZero = Eigen::Matrix4d::Identity();
		mMoveToPartZero(0, 3) = -modelX;
		mMoveToPartZero(1, 3) = -modelY;
		mMoveToPartZero(2, 3) = -modelZ;

		// 最终变换：工件坐标系 = mMoveToPartZero * inv(mRobotMotion) * handEye
		const Eigen::Matrix4d finalMatrix =
		    mMoveToPartZero * mRobotMotion.inverse() * m_pointCloudService->getHandEyeMatrix();

		cloud->setGLTransformation(ccGLMatrix(finalMatrix.data()));
		cloud->applyGLTransformation_recursive();
		cloud->prepareDisplayForRefresh_recursive();
	}

	


	m_app->addToDB(cloud);
	m_app->refreshAll();
	m_app->updateUI();

	return true;
}
