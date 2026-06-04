#include "SparkMachineProgramDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

SparkMachineProgramDialog::SparkMachineProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("创建火花机程序");
	setFixedSize(550, 900);
	init();
}

SparkMachineProgramDialog::~SparkMachineProgramDialog()
{
}

void SparkMachineProgramDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(20, 16, 20, 16);
	m_mainLayout->setSpacing(12);

	QLabel* headerLabel = new QLabel("火花机程序参数", this);
	QFont font = headerLabel->font();
	font.setBold(true);
	font.setPointSize(font.pointSize() + 1);
	headerLabel->setFont(font);
	m_mainLayout->addWidget(headerLabel);

	QWidget* formCard = new QWidget(this);
	formCard->setStyleSheet(R"(
		QWidget {
			background-color: #f7f7f9;
			border: 1px solid #d0d0d0;
			border-radius: 4px;
		}
		QComboBox, QLineEdit, QDoubleSpinBox {
			background-color: #ffffff;
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 2px 6px;
			min-height: 26px;
		}
		QComboBox:focus, QLineEdit:focus, QDoubleSpinBox:focus {
			border: 1px solid #4a90d9;
		}
	)");

	QVBoxLayout* cardLayout = new QVBoxLayout(formCard);
	cardLayout->setContentsMargins(16, 14, 16, 14);
	cardLayout->setSpacing(12);

	QFormLayout* basicLayout = new QFormLayout();
	basicLayout->setVerticalSpacing(10);
	basicLayout->setHorizontalSpacing(16);
	basicLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

	QLabel* machineTypeLabel = new QLabel("机床类型：");
	machineTypeLabel->setStyleSheet("border: none; background: transparent;");
	m_machineTypeCombo = new QComboBox(this);
	m_machineTypeCombo->addItem("DIMENG", "DIMENG");
	m_machineTypeCombo->addItem("ONA", "ONA");
	basicLayout->addRow(machineTypeLabel, m_machineTypeCombo);

	QLabel* partRfidLabel = new QLabel("工件RFID：");
	partRfidLabel->setStyleSheet("border: none; background: transparent;");
	m_partRfidEdit = new QLineEdit(this);
	m_partRfidEdit->setPlaceholderText("请输入工件RFID编号");
	basicLayout->addRow(partRfidLabel, m_partRfidEdit);

	QLabel* electrodeRfidLabel = new QLabel("电极RFID：");
	electrodeRfidLabel->setStyleSheet("border: none; background: transparent;");
	m_electrodeRfidEdit = new QLineEdit(this);
	m_electrodeRfidEdit->setPlaceholderText("请输入电极RFID编号");
	basicLayout->addRow(electrodeRfidLabel, m_electrodeRfidEdit);

	QLabel* partTypeLabel = new QLabel("工件类型：");
	partTypeLabel->setStyleSheet("border: none; background: transparent;");
	m_partTypeEdit = new QLineEdit(this);
	m_partTypeEdit->setPlaceholderText("请输入工件类型");
	basicLayout->addRow(partTypeLabel, m_partTypeEdit);

	QLabel* electrodeTypeLabel = new QLabel("电极类型：");
	electrodeTypeLabel->setStyleSheet("border: none; background: transparent;");
	m_electrodeTypeEdit = new QLineEdit(this);
	m_electrodeTypeEdit->setPlaceholderText("请输入电极类型");
	basicLayout->addRow(electrodeTypeLabel, m_electrodeTypeEdit);

	cardLayout->addLayout(basicLayout);

	QGroupBox* uvwGroup = new QGroupBox("UVW旋转中心", this);
	QFormLayout* uvwLayout = new QFormLayout(uvwGroup);
	uvwLayout->setVerticalSpacing(8);
	uvwLayout->setHorizontalSpacing(12);

	m_uvwCenterXSpin = new QDoubleSpinBox(this);
	m_uvwCenterXSpin->setRange(-9999.999, 9999.999);
	m_uvwCenterXSpin->setDecimals(3);
	m_uvwCenterXSpin->setValue(0.0);

	m_uvwCenterYSpin = new QDoubleSpinBox(this);
	m_uvwCenterYSpin->setRange(-9999.999, 9999.999);
	m_uvwCenterYSpin->setDecimals(3);
	m_uvwCenterYSpin->setValue(0.0);

	m_uvwCenterZSpin = new QDoubleSpinBox(this);
	m_uvwCenterZSpin->setRange(-9999.999, 9999.999);
	m_uvwCenterZSpin->setDecimals(3);
	m_uvwCenterZSpin->setValue(0.0);

	uvwLayout->addRow("X:", m_uvwCenterXSpin);
	uvwLayout->addRow("Y:", m_uvwCenterYSpin);
	uvwLayout->addRow("Z:", m_uvwCenterZSpin);
	cardLayout->addWidget(uvwGroup);

	QGroupBox* topChuckGroup = new QGroupBox("上卡盘中心", this);
	QFormLayout* topChuckLayout = new QFormLayout(topChuckGroup);
	topChuckLayout->setVerticalSpacing(8);
	topChuckLayout->setHorizontalSpacing(12);

	m_topChuckCenterXSpin = new QDoubleSpinBox(this);
	m_topChuckCenterXSpin->setRange(-9999.999, 9999.999);
	m_topChuckCenterXSpin->setDecimals(3);
	m_topChuckCenterXSpin->setValue(0.0);

	m_topChuckCenterYSpin = new QDoubleSpinBox(this);
	m_topChuckCenterYSpin->setRange(-9999.999, 9999.999);
	m_topChuckCenterYSpin->setDecimals(3);
	m_topChuckCenterYSpin->setValue(0.0);

	m_topChuckCenterZSpin = new QDoubleSpinBox(this);
	m_topChuckCenterZSpin->setRange(-9999.999, 9999.999);
	m_topChuckCenterZSpin->setDecimals(3);
	m_topChuckCenterZSpin->setValue(0.0);

	topChuckLayout->addRow("X:", m_topChuckCenterXSpin);
	topChuckLayout->addRow("Y:", m_topChuckCenterYSpin);
	topChuckLayout->addRow("Z:", m_topChuckCenterZSpin);
	cardLayout->addWidget(topChuckGroup);

	QGroupBox* bottomChuckGroup = new QGroupBox("下卡盘中心", this);
	QFormLayout* bottomChuckLayout = new QFormLayout(bottomChuckGroup);
	bottomChuckLayout->setVerticalSpacing(8);
	bottomChuckLayout->setHorizontalSpacing(12);

	m_bottomChuckCenterXSpin = new QDoubleSpinBox(this);
	m_bottomChuckCenterXSpin->setRange(-9999.999, 9999.999);
	m_bottomChuckCenterXSpin->setDecimals(3);
	m_bottomChuckCenterXSpin->setValue(0.0);

	m_bottomChuckCenterYSpin = new QDoubleSpinBox(this);
	m_bottomChuckCenterYSpin->setRange(-9999.999, 9999.999);
	m_bottomChuckCenterYSpin->setDecimals(3);
	m_bottomChuckCenterYSpin->setValue(0.0);

	m_bottomChuckCenterZSpin = new QDoubleSpinBox(this);
	m_bottomChuckCenterZSpin->setRange(-9999.999, 9999.999);
	m_bottomChuckCenterZSpin->setDecimals(3);
	m_bottomChuckCenterZSpin->setValue(0.0);

	bottomChuckLayout->addRow("X:", m_bottomChuckCenterXSpin);
	bottomChuckLayout->addRow("Y:", m_bottomChuckCenterYSpin);
	bottomChuckLayout->addRow("Z:", m_bottomChuckCenterZSpin);
	cardLayout->addWidget(bottomChuckGroup);

	m_mainLayout->addWidget(formCard);

	m_mainLayout->addStretch();

	QFrame* separator = new QFrame(this);
	separator->setFrameShape(QFrame::HLine);
	separator->setFrameShadow(QFrame::Sunken);
	m_mainLayout->addWidget(separator);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->setSpacing(8);
	buttonLayout->addStretch();

	QPushButton* cancelButton = new QPushButton("取消", this);
	cancelButton->setFixedWidth(80);
	cancelButton->setFixedHeight(32);
	connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
	buttonLayout->addWidget(cancelButton);

	QPushButton* startButton = new QPushButton("确认创建", this);
	startButton->setFixedWidth(100);
	startButton->setFixedHeight(32);
	startButton->setDefault(true);
	connect(startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);
	buttonLayout->addWidget(startButton);

	m_mainLayout->addLayout(buttonLayout);
}

void SparkMachineProgramDialog::onOperationStarted()
{
	setProgressText("正在创建火花机程序...");
}

bool SparkMachineProgramDialog::performOperation()
{
	QString machineType = m_machineTypeCombo->currentData().toString();
	QString partRfid = m_partRfidEdit->text().trimmed();
	QString electrodeRfid = m_electrodeRfidEdit->text().trimmed();
	QString partType = m_partTypeEdit->text().trimmed();
	QString electrodeType = m_electrodeTypeEdit->text().trimmed();

	if (partRfid.isEmpty()) {
		m_lastError = "请输入工件RFID";
		return false;
	}

	if (electrodeRfid.isEmpty()) {
		m_lastError = "请输入电极RFID";
		return false;
	}

	if (partType.isEmpty()) {
		m_lastError = "请输入工件类型";
		return false;
	}

	if (electrodeType.isEmpty()) {
		m_lastError = "请输入电极类型";
		return false;
	}

	QJsonObject edmParams;
	QJsonObject uvwCenter;
	uvwCenter["X"] = m_uvwCenterXSpin->value();
	uvwCenter["Y"] = m_uvwCenterYSpin->value();
	uvwCenter["Z"] = m_uvwCenterZSpin->value();
	edmParams["UVW_Center"] = uvwCenter;

	QJsonObject topChuckCenter;
	topChuckCenter["X"] = m_topChuckCenterXSpin->value();
	topChuckCenter["Y"] = m_topChuckCenterYSpin->value();
	topChuckCenter["Z"] = m_topChuckCenterZSpin->value();
	edmParams["TopChuck_Center"] = topChuckCenter;

	QJsonObject bottomChuckCenter;
	bottomChuckCenter["X"] = m_bottomChuckCenterXSpin->value();
	bottomChuckCenter["Y"] = m_bottomChuckCenterYSpin->value();
	bottomChuckCenter["Z"] = m_bottomChuckCenterZSpin->value();
	edmParams["BottomChuck_Center"] = bottomChuckCenter;

	return m_pointCloudService->executeSparkMachineProgram(
		machineType,
		partType,
		electrodeType,
		partRfid,
		electrodeRfid,
		edmParams,
		m_lastError
	);
}

void SparkMachineProgramDialog::onOperationCompleted(bool success)
{
	if (success) {
		setProgressText("✅ 火花机程序创建完成");
		accept();
	}
	else {
		setProgressText(QString("❌ 火花机程序创建失败：%1").arg(m_lastError));
	}
}
