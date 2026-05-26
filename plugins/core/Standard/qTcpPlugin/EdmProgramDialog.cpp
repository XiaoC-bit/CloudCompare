#include "EdmProgramDialog.h"
#include "PointCloudService.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <ccMainAppInterface.h>

EdmProgramDialog::EdmProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("生成放电程序");
	setMinimumWidth(400);
	init();
}

EdmProgramDialog::~EdmProgramDialog()
{
}

void EdmProgramDialog::setupAdditionalUI()
{
	QFrame* inputFrame = new QFrame(this);
	inputFrame->setFrameShape(QFrame::NoFrame);
	inputFrame->setStyleSheet(R"(
		QFrame {
			background-color: #ffffff;
			border: 1px solid #d0d0d0;
			border-radius: 4px;
		}
	)");

	QVBoxLayout* frameLayout = new QVBoxLayout(inputFrame);
	frameLayout->setContentsMargins(16, 12, 16, 12);
	frameLayout->setSpacing(10);

	QGridLayout* gridLayout = new QGridLayout();
	gridLayout->setHorizontalSpacing(12);
	gridLayout->setVerticalSpacing(10);

	QLabel* xLabel = new QLabel("X:");
	xLabel->setStyleSheet("color: #444444;");
	m_xSpinBox = new QDoubleSpinBox();
	m_xSpinBox->setRange(-9999.999, 9999.999);
	m_xSpinBox->setDecimals(3);
	m_xSpinBox->setFixedWidth(150);
	m_xSpinBox->setStyleSheet(R"(
		QDoubleSpinBox {
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 4px;
		}
	)");
	gridLayout->addWidget(xLabel, 0, 0);
	gridLayout->addWidget(m_xSpinBox, 0, 1);

	QLabel* yLabel = new QLabel("Y:");
	yLabel->setStyleSheet("color: #444444;");
	m_ySpinBox = new QDoubleSpinBox();
	m_ySpinBox->setRange(-9999.999, 9999.999);
	m_ySpinBox->setDecimals(3);
	m_ySpinBox->setFixedWidth(150);
	m_ySpinBox->setStyleSheet(R"(
		QDoubleSpinBox {
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 4px;
		}
	)");
	gridLayout->addWidget(yLabel, 1, 0);
	gridLayout->addWidget(m_ySpinBox, 1, 1);

	QLabel* zLabel = new QLabel("Z:");
	zLabel->setStyleSheet("color: #444444;");
	m_zSpinBox = new QDoubleSpinBox();
	m_zSpinBox->setRange(-9999.999, 9999.999);
	m_zSpinBox->setDecimals(3);
	m_zSpinBox->setFixedWidth(150);
	m_zSpinBox->setStyleSheet(R"(
		QDoubleSpinBox {
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 4px;
		}
	)");
	gridLayout->addWidget(zLabel, 2, 0);
	gridLayout->addWidget(m_zSpinBox, 2, 1);

	QLabel* bLabel = new QLabel("B:");
	bLabel->setStyleSheet("color: #444444;");
	m_bSpinBox = new QDoubleSpinBox();
	m_bSpinBox->setRange(-180, 180);
	m_bSpinBox->setDecimals(3);
	m_bSpinBox->setFixedWidth(150);
	m_bSpinBox->setStyleSheet(R"(
		QDoubleSpinBox {
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 4px;
		}
	)");
	gridLayout->addWidget(bLabel, 3, 0);
	gridLayout->addWidget(m_bSpinBox, 3, 1);

	QLabel* cLabel = new QLabel("C:");
	cLabel->setStyleSheet("color: #444444;");
	m_cSpinBox = new QDoubleSpinBox();
	m_cSpinBox->setRange(-180, 180);
	m_cSpinBox->setDecimals(3);
	m_cSpinBox->setFixedWidth(150);
	m_cSpinBox->setStyleSheet(R"(
		QDoubleSpinBox {
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 4px;
		}
	)");
	gridLayout->addWidget(cLabel, 4, 0);
	gridLayout->addWidget(m_cSpinBox, 4, 1);

	frameLayout->addLayout(gridLayout);

	m_mainLayout->addWidget(inputFrame);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->setSpacing(10);
	buttonLayout->addStretch();

	QPushButton* startButton = new QPushButton("确认执行");
	startButton->setFixedHeight(32);
	startButton->setFixedWidth(100);
	startButton->setStyleSheet(R"(
		QPushButton {
			background-color: #3366cc;
			color: white;
			border: none;
			border-radius: 4px;
			font-weight: bold;
		}
		QPushButton:hover {
			background-color: #2550a8;
		}
		QPushButton:disabled {
			background-color: #a0a0a0;
		}
	)");
	connect(startButton, &QPushButton::clicked, this, &EdmProgramDialog::onStartOperation);
	buttonLayout->addWidget(startButton);

	QPushButton* cancelButton = new QPushButton("取消");
	cancelButton->setFixedHeight(32);
	cancelButton->setFixedWidth(100);
	cancelButton->setStyleSheet(R"(
		QPushButton {
			background-color: #f0f0f0;
			color: #444444;
			border: 1px solid #d0d0d0;
			border-radius: 4px;
		}
		QPushButton:hover {
			background-color: #e0e0e0;
		}
	)");
	connect(cancelButton, &QPushButton::clicked, this, &EdmProgramDialog::reject);
	buttonLayout->addWidget(cancelButton);

	m_mainLayout->addLayout(buttonLayout);

	m_mainLayout->setSpacing(16);
	m_mainLayout->setContentsMargins(20, 20, 20, 20);
}

void EdmProgramDialog::onOperationStarted()
{
	setProgressText("正在生成放电程序...");
}

bool EdmProgramDialog::performOperation()
{
	double x = m_xSpinBox->value();
	double y = m_ySpinBox->value();
	double z = m_zSpinBox->value();
	double b = m_bSpinBox->value();
	double c = m_cSpinBox->value();

	return m_pointCloudService->executeEdmProgram(x, y, z, b, c);
}

void EdmProgramDialog::onOperationCompleted(bool success)
{
	if (success) {
		setProgressText("放电程序执行完成");
	} else {
		setProgressText("放电程序执行失败");
	}
}
