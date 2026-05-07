#include "ElectrodeInspectDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>

ElectrodeInspectDialog::ElectrodeInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("电极检测");
	setFixedSize(450, 250);
}

ElectrodeInspectDialog::~ElectrodeInspectDialog()
{
}

void ElectrodeInspectDialog::setupAdditionalUI()
{
	QFormLayout* formLayout = new QFormLayout();

	m_electrodeTypeCombo = new QComboBox(this);
	m_electrodeTypeCombo->addItem("ElectrodeA");
	m_electrodeTypeCombo->addItem("ElectrodeB");
	m_electrodeTypeCombo->addItem("ElectrodeC");
	m_electrodeTypeCombo->addItem("ElectrodeD");
	m_electrodeTypeCombo->addItem("ElectrodeE");
	formLayout->addRow("电极类型：", m_electrodeTypeCombo);

	m_rfidEdit = new QLineEdit(this);
	formLayout->addRow("RFID：", m_rfidEdit);

	m_mainLayout->insertLayout(1, formLayout);

	m_mainLayout->addStretch();
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
