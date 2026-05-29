#include "RingCalibrationDialog.h"
#include "PointCloudService.h"
#include <QHBoxLayout>
#include <QPushButton>
#include <qevent.h>

RingCalibrationDialog::RingCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("环规标定");
	setFixedSize(450, 200);
	init();
}

RingCalibrationDialog::~RingCalibrationDialog()
{
}

void RingCalibrationDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(24, 16, 24, 20);
	m_mainLayout->setSpacing(16);

	m_mainLayout->addStretch();

	m_buttonLayout = new QHBoxLayout();
	m_buttonLayout->setSpacing(12);

	m_buttonLayout->addStretch();

	m_startButton = new QPushButton("开始标定", this);
	m_startButton->setFixedWidth(110);
	m_startButton->setDefault(true);
	connect(m_startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);

	m_cancelButton = new QPushButton("取消", this);
	m_cancelButton->setFixedWidth(90);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

	m_buttonLayout->addWidget(m_cancelButton);
	m_buttonLayout->addWidget(m_startButton);

	m_mainLayout->addLayout(m_buttonLayout);
}

void RingCalibrationDialog::onOperationStarted()
{
	setProgressText("开始环规标定...");
}

bool RingCalibrationDialog::performOperation()
{
	return m_pointCloudService->executeRingCalibration();
}

void RingCalibrationDialog::onOperationCompleted(bool success)
{
	if (success)
	{
		setProgressText(QString("✅ 环规标定完成"));
	}
	else
	{
		setProgressText(QString("❌ 环规标定失败"));
	}
}
