#include "PartInspectDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>

PartInspectDialog::PartInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("工件检测");
	setFixedSize(450, 250);
}

PartInspectDialog::~PartInspectDialog()
{
}

void PartInspectDialog::setupAdditionalUI()
{
	QFormLayout* formLayout = new QFormLayout();

	m_partTypeCombo = new QComboBox(this);
	m_partTypeCombo->addItem("PartA");
	m_partTypeCombo->addItem("PartB");
	m_partTypeCombo->addItem("PartC");
	formLayout->addRow("工件类型：", m_partTypeCombo);

	m_rfidEdit = new QLineEdit(this);
	formLayout->addRow("RFID：", m_rfidEdit);

	m_mainLayout->insertLayout(1, formLayout);

	m_mainLayout->addStretch();
}

void PartInspectDialog::onOperationStarted()
{
	setProgressText("开始工件检测...");
}

bool PartInspectDialog::performOperation()
{
	QString partType = m_partTypeCombo->currentText();
	QString rfid = m_rfidEdit->text();

	if (rfid.isEmpty()) {
		setProgressText("❌ 请输入RFID");
		return false;
	}

	return m_pointCloudService->executePartInspect(partType, rfid);
}

void PartInspectDialog::onOperationCompleted(bool success)
{
	if (success) {
		QJsonObject result = m_pointCloudService->getPartInspectResult();
		QJsonObject inspectResult = result["InspectResult"].toObject();
		if (inspectResult["Result"].toString() == "OK") {
			setProgressText("✅ 工件检测完成");
			accept();
		}
		else {
			setProgressText(QString("❌ 工件检测失败：%1").arg(inspectResult["Ret_Err"].toString()));
		}
	}
	else {
		QJsonObject result = m_pointCloudService->getPartInspectResult();
		QJsonObject inspectResult = result["InspectResult"].toObject();
		setProgressText(QString("❌ 工件检测失败：%1").arg(inspectResult["Ret_Err"].toString()));
	}
}
