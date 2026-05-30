#include "RingCalibrationDialog.h"
#include "PointCloudService.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QFrame>
#include <qevent.h>

RingCalibrationDialog::RingCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("环规标定");
	setFixedSize(550, 380);
	init();
}

RingCalibrationDialog::~RingCalibrationDialog()
{
}

void RingCalibrationDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(20, 16, 20, 20);
	m_mainLayout->setSpacing(12);

	m_infoFrame = new QFrame(this);
	m_infoFrame->setFrameShape(QFrame::StyledPanel);
	m_infoFrame->setStyleSheet("QFrame { background-color: #f5f7fa; border: 1px solid #d0d7e0; border-radius: 8px; }");
	QHBoxLayout* infoLayout = new QHBoxLayout(m_infoFrame);
	infoLayout->setContentsMargins(16, 16, 16, 16);
	infoLayout->setSpacing(20);

	m_schematicLabel = new QLabel(this);
	m_schematicLabel->setFixedSize(280, 140);
	m_schematicLabel->setStyleSheet("QLabel { background-color: white; border: 2px solid #c0c8d0; border-radius: 6px; }");
	m_schematicLabel->setAlignment(Qt::AlignCenter);
	m_schematicLabel->setScaledContents(true);

	QPixmap schematicPixmap(":/CC/plugin/qTcpPlugin/res/ring_op.png");
	if (!schematicPixmap.isNull()) {
		m_schematicLabel->setPixmap(schematicPixmap.scaled(280, 140, Qt::KeepAspectRatio, Qt::SmoothTransformation));
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
		"<p style='margin: 4px 0;'>1. 请将<strong>环规</strong>放置在<strong>机床工作台</strong></p>"
		"<p style='margin: 4px 0;'>2. 确保环规表面<strong>干净</strong></p>"
		"<p style='margin: 10px 0 0 0; color: #cc6600; font-weight: bold;'>⚠️ 确认以上步骤完成后，再点击「开始标定」</p>"
	);

	infoLayout->addWidget(m_instructionLabel, 1);

	m_mainLayout->insertWidget(4, m_infoFrame);

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
