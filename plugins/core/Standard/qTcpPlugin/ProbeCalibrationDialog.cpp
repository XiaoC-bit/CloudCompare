#include "ProbeCalibrationDialog.h"
#include "PointCloudService.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <ccMainAppInterface.h>
#include <QTimer>
#include <qevent.h>

ProbeCalibrationDialog::ProbeCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : QDialog(parent)
    , m_pointCloudService(pointCloudService)
    , m_app(app)
{
	setWindowTitle("测头标定");
	setFixedSize(450, 200);
	setupUI();
}

ProbeCalibrationDialog::~ProbeCalibrationDialog()
{
}

void ProbeCalibrationDialog::setupUI()
{
	m_mainLayout = new QVBoxLayout(this);

	m_progressLabel = new QLabel("准备就绪", this);
	m_progressLabel->setAlignment(Qt::AlignCenter);
	m_mainLayout->addWidget(m_progressLabel);

	m_progressBar = new QProgressBar(this);
	m_progressBar->setRange(0, 100);
	m_progressBar->setValue(0);
	m_progressBar->setVisible(false);
	m_mainLayout->addWidget(m_progressBar);

	m_mainLayout->addStretch();

	m_buttonLayout = new QHBoxLayout();
	m_startButton = new QPushButton("开始标定", this);
	connect(m_startButton, &QPushButton::clicked, this, &ProbeCalibrationDialog::onStartCalibration);
	m_cancelButton = new QPushButton("取消", this);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

	m_buttonLayout->addWidget(m_startButton);
	m_buttonLayout->addWidget(m_cancelButton);
	m_mainLayout->addLayout(m_buttonLayout);
}

void ProbeCalibrationDialog::closeEvent(QCloseEvent* event)
{
	if (m_calibrationRunning)
	{
		event->ignore();
	}
	else
	{
		QDialog::closeEvent(event);
	}
}

void ProbeCalibrationDialog::setCalibrationRunning(bool running)
{
	m_calibrationRunning = running;

	m_startButton->setEnabled(!running);
	m_cancelButton->setEnabled(!running);

	m_progressBar->setVisible(running);
}

void ProbeCalibrationDialog::onStartCalibration()
{
	if (!m_pointCloudService)
	{
		m_progressLabel->setText("❌ 错误：PointCloudService未初始化");
		return;
	}

	ProbeCalibrationGuard guard(this);
	m_progressBar->setVisible(true);
	m_progressBar->setValue(0);
	m_progressLabel->setText("开始测头标定...");

	m_pointCloudService->executeProbeCalibration();

	m_progressBar->setValue(100);

	QJsonObject result = m_pointCloudService->getProbeCalibrationResult();
	QJsonObject calibResult = result["CalibrationResult"].toObject();
	if (calibResult["Result"].toString() == "OK")
	{
		QJsonObject rotationCenter = calibResult["RotationCenter"].toObject();
		m_progressLabel->setText(QString("✅ 测头标定完成\n旋转中心: X=%1 Y=%2 Z=%3")
		                             .arg(rotationCenter["X"].toDouble(), 0, 'f', 3)
		                             .arg(rotationCenter["Y"].toDouble(), 0, 'f', 3)
		                             .arg(rotationCenter["Z"].toDouble(), 0, 'f', 3));
		accept();
	}
	else
	{
		m_progressLabel->setText(QString("❌ 测头标定失败: %1").arg(calibResult["Ret_Err"].toString()));
	}
}
