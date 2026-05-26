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
}

EdmProgramDialog::~EdmProgramDialog()
{
}

void EdmProgramDialog::setupAdditionalUI()
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setContentsMargins(20, 20, 20, 20);
	mainLayout->setSpacing(16);

	QGridLayout* gridLayout = new QGridLayout();
	gridLayout->setHorizontalSpacing(12);
	gridLayout->setVerticalSpacing(10);

	QLabel* xLabel = new QLabel("X:");
	xLabel->setStyleSheet("font-weight: bold;");
	m_xSpinBox = new QDoubleSpinBox();
	m_xSpinBox->setRange(-9999.999, 9999.999);
	m_xSpinBox->setDecimals(3);
	m_xSpinBox->setFixedWidth(150);
	gridLayout->addWidget(xLabel, 0, 0);
	gridLayout->addWidget(m_xSpinBox, 0, 1);

	QLabel* yLabel = new QLabel("Y:");
	yLabel->setStyleSheet("font-weight: bold;");
	m_ySpinBox = new QDoubleSpinBox();
	m_ySpinBox->setRange(-9999.999, 9999.999);
	m_ySpinBox->setDecimals(3);
	m_ySpinBox->setFixedWidth(150);
	gridLayout->addWidget(yLabel, 1, 0);
	gridLayout->addWidget(m_ySpinBox, 1, 1);

	QLabel* zLabel = new QLabel("Z:");
	zLabel->setStyleSheet("font-weight: bold;");
	m_zSpinBox = new QDoubleSpinBox();
	m_zSpinBox->setRange(-9999.999, 9999.999);
	m_zSpinBox->setDecimals(3);
	m_zSpinBox->setFixedWidth(150);
	gridLayout->addWidget(zLabel, 2, 0);
	gridLayout->addWidget(m_zSpinBox, 2, 1);

	QLabel* bLabel = new QLabel("B:");
	bLabel->setStyleSheet("font-weight: bold;");
	m_bSpinBox = new QDoubleSpinBox();
	m_bSpinBox->setRange(-180, 180);
	m_bSpinBox->setDecimals(3);
	m_bSpinBox->setFixedWidth(150);
	gridLayout->addWidget(bLabel, 3, 0);
	gridLayout->addWidget(m_bSpinBox, 3, 1);

	QLabel* cLabel = new QLabel("C:");
	cLabel->setStyleSheet("font-weight: bold;");
	m_cSpinBox = new QDoubleSpinBox();
	m_cSpinBox->setRange(-180, 180);
	m_cSpinBox->setDecimals(3);
	m_cSpinBox->setFixedWidth(150);
	gridLayout->addWidget(cLabel, 4, 0);
	gridLayout->addWidget(m_cSpinBox, 4, 1);

	mainLayout->addLayout(gridLayout);
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
