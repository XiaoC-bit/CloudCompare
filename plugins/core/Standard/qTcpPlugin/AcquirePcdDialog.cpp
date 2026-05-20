#include "AcquirePcdDialog.h"
#include <QMessageBox>
#include <QThread>
#include <qeventloop.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccMainAppInterface.h>
#include <ccHObject.h>


#include "LJS8_IF.h"
#include "LJS8_ErrorCode.h"
#include "LJS8_ACQ.h"

AcquirePcdDialog::AcquirePcdDialog(ccMainAppInterface* app, QWidget* parent)
    : QDialog(parent)
    , m_app(app)
    , m_isAcquiring(false)
    , m_cancelRequested(false)
{
    setWindowTitle("获取点云");
    setFixedSize(400, 200);
    setModal(true);
    setupUI();
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
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(15);

    m_statusLabel = new QLabel("准备获取点云", this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(m_statusLabel);

    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 100);
    m_progressBar->setValue(0);
    m_progressBar->setVisible(false);
    mainLayout->addWidget(m_progressBar);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);

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

    QThread::create([this]() {
        bool success = acquirePointCloud("AcquiredCloud");
        QString errorMessage;

        if (!success && !m_cancelRequested) {
            errorMessage = "点云获取失败";
        }

        QMetaObject::invokeMethod(this, "onAcquisitionFinished",
            Qt::QueuedConnection,
            Q_ARG(bool, success),
            Q_ARG(QString, errorMessage));
    })->start();
}

void AcquirePcdDialog::onCancelClicked()
{
    m_cancelRequested = true;
    m_statusLabel->setText("正在取消...");
    m_cancelBtn->setEnabled(false);
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
			if (heightBuf[idx] != 0)
			{
				CCVector3 P(
                     static_cast<PointCoordinateType>(x * xPitch),
			    static_cast<PointCoordinateType>(y * yPitch),
			    static_cast<PointCoordinateType>((heightBuf[idx] - 32768) * zPitch));
				cloud->addPoint(P);
			}
		}
	}

	m_app->addToDB(cloud);
	m_app->refreshAll();
	m_app->updateUI();

	return true;
}
