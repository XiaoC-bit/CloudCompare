#include "ProbeCalibrationDialog.h"
#include "PointCloudService.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <qevent.h>

ProbeCalibrationDialog::ProbeCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("测头标定");
	setFixedSize(450, 200);
	init();
}

ProbeCalibrationDialog::~ProbeCalibrationDialog()
{
}

void ProbeCalibrationDialog::setupAdditionalUI()
{
	m_mainLayout->addStretch();

	m_buttonLayout = new QHBoxLayout();
	m_startButton = new QPushButton("开始标定", this);
	connect(m_startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);
	m_cancelButton = new QPushButton("取消", this);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

	m_buttonLayout->addWidget(m_startButton);
	m_buttonLayout->addWidget(m_cancelButton);
	m_mainLayout->addLayout(m_buttonLayout);
}

void ProbeCalibrationDialog::onOperationStarted()
{
	setProgressText("开始测头标定...");
}

bool ProbeCalibrationDialog::performOperation()
{
	return m_pointCloudService->executeProbeCalibration();
}

void ProbeCalibrationDialog::onOperationCompleted(bool success)
{
	if (success)
	{
		QJsonObject result = m_pointCloudService->getProbeCalibrationResult();
		QJsonObject calibResult = result["CalibrationResult"].toObject();
		QJsonObject rotationCenter = calibResult["RotationCenter"].toObject();
		setProgressText(QString("✅ 测头标定完成\n旋转中心: X=%1 Y=%2 Z=%3")
		                     .arg(rotationCenter["X"].toDouble(), 0, 'f', 3)
		                     .arg(rotationCenter["Y"].toDouble(), 0, 'f', 3)
		                     .arg(rotationCenter["Z"].toDouble(), 0, 'f', 3));
		accept();
	}
	else
	{
		QJsonObject result = m_pointCloudService->getProbeCalibrationResult();
		QJsonObject calibResult = result["CalibrationResult"].toObject();
		setProgressText(QString("❌ 测头标定失败: %1").arg(calibResult["Ret_Err"].toString()));
	}
}
