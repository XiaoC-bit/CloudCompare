#include "ElectrodeInspectDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>

ElectrodeInspectDialog::ElectrodeInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("电极检测");
	setFixedSize(450, 320);
	init();
}

ElectrodeInspectDialog::~ElectrodeInspectDialog()
{
}

void ElectrodeInspectDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(24, 16, 24, 20);
	m_mainLayout->setSpacing(16);

	QGroupBox* formGroup = new QGroupBox("检测参数", this);
	QFormLayout* formLayout = new QFormLayout(formGroup);
	formLayout->setContentsMargins(16, 12, 16, 12);
	formLayout->setVerticalSpacing(12);
	formLayout->setHorizontalSpacing(16);
	formLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

	m_electrodeTypeCombo = new QComboBox(this);
	m_electrodeTypeCombo->setMinimumHeight(28);
	m_electrodeTypeCombo->addItem("ElectrodeA");
	m_electrodeTypeCombo->addItem("ElectrodeB");
	m_electrodeTypeCombo->addItem("ElectrodeC");
	m_electrodeTypeCombo->addItem("ElectrodeD");
	m_electrodeTypeCombo->addItem("ElectrodeE");
	formLayout->addRow("电极类型：", m_electrodeTypeCombo);

	m_rfidEdit = new QLineEdit(this);
	m_rfidEdit->setMinimumHeight(28);
	m_rfidEdit->setPlaceholderText("请输入RFID编号");
	formLayout->addRow("RFID：", m_rfidEdit);

	m_mainLayout->addWidget(formGroup);

	m_mainLayout->addStretch();

	m_buttonLayout = new QHBoxLayout();
	m_buttonLayout->setSpacing(12);

	m_buttonLayout->addStretch();

	m_startButton = new QPushButton("开始检测", this);
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

void ElectrodeInspectDialog::onOperationStarted()
{
	setProgressText("开始电极检测...");
}

bool ElectrodeInspectDialog::performOperation()
{
	QString electrodeType = m_electrodeTypeCombo->currentText();
	QString rfid = m_rfidEdit->text();

	if (rfid.isEmpty()) {
		setProgressText("❌ 请输入RFID");
		return false;
	}

	return m_pointCloudService->executeElectrodeInspect(electrodeType, rfid);
}

void ElectrodeInspectDialog::onOperationCompleted(bool success)
{
	if (success) {
		QJsonObject result = m_pointCloudService->getElectrodeInspectResult();
		QJsonObject inspectResult = result["InspectResult"].toObject();
		if (inspectResult["Result"].toString() == "OK") {
			setProgressText("✅ 电极检测完成");
			accept();
		}
		else {
			setProgressText(QString("❌ 电极检测失败：%1").arg(inspectResult["Ret_Err"].toString()));
		}
	}
	else {
		QJsonObject result = m_pointCloudService->getElectrodeInspectResult();
		QJsonObject inspectResult = result["InspectResult"].toObject();
		setProgressText(QString("❌ 电极检测失败：%1").arg(inspectResult["Ret_Err"].toString()));
	}
}
