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
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QMap>
#include <QFrame>

SparkMachineProgramDialog::SparkMachineProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("创建火花机程序");
	setMinimumSize(1000, 700);
	resize(1100, 750);
	init();
	loadPartTypes();
	loadConfig();
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
		QLabel {
			border: none;
			background: transparent;
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
		QGroupBox {
			font-weight: bold;
			margin-top: 6px;
			padding-top: 14px;
			border: 1px solid #c8c8c8;
			border-radius: 4px;
		}
		QGroupBox::title {
			subcontrol-origin: margin;
			subcontrol-position: top left;
			left: 8px;
			top: 2px;
			padding: 0 4px;
		}
	)");

	QHBoxLayout* mainCardLayout = new QHBoxLayout(formCard);
	mainCardLayout->setContentsMargins(16, 14, 16, 14);
	mainCardLayout->setSpacing(16);

	// 左侧 - 基本信息
	QVBoxLayout* leftLayout = new QVBoxLayout();
	leftLayout->setSpacing(10);

	QLabel* basicInfoLabel = new QLabel("基本信息", this);
	QFont labelFont = basicInfoLabel->font();
	labelFont.setBold(true);
	basicInfoLabel->setFont(labelFont);
	leftLayout->addWidget(basicInfoLabel);

	QFormLayout* basicLayout = new QFormLayout();
	basicLayout->setVerticalSpacing(10);
	basicLayout->setHorizontalSpacing(12);
	basicLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

	QLabel* machineTypeLabel = new QLabel("机床类型：");
		machineTypeLabel->setStyleSheet("border: none; background: transparent;");
		m_machineTypeCombo = new QComboBox(this);
		m_machineTypeCombo->addItem("DIMENG", "DIMENG");
		m_machineTypeCombo->addItem("ONA", "ONA");
		basicLayout->addRow(machineTypeLabel, m_machineTypeCombo);

		QLabel* machineNameLabel = new QLabel("机床名称：");
		machineNameLabel->setStyleSheet("border: none; background: transparent;");
		m_machineNameCombo = new QComboBox(this);
		m_machineNameCombo->setStyleSheet(R"(
			QComboBox {
				background-color: #ffffff;
				border: 1px solid #d0d0d0;
				border-radius: 3px;
				padding: 2px 6px;
				min-height: 26px;
			}
			QComboBox:focus {
				border: 1px solid #4a90d9;
			}
			QComboBox QAbstractItemView {
				border: 1px solid #d0d0d0;
				outline: none;
			}
			QComboBox QAbstractItemView::item {
				min-height: 26px;
				padding: 0 8px;
			}
		)");
		basicLayout->addRow(machineNameLabel, m_machineNameCombo);

		m_machineNameMap["DIMENG"] = QStringList() << "DIMENG-001" << "DIMENG-002" << "DIMENG-003" << "DIMENG-004" << "DIMENG-005";
		m_machineNameMap["ONA"] = QStringList() << "ONA-001" << "ONA-002" << "ONA-003" << "ONA-004" << "ONA-005";

		updateMachineNames();

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
	m_partTypeCombo = new QComboBox(this);
	m_partTypeCombo->setStyleSheet(R"(
		QComboBox {
			background-color: #ffffff;
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 2px 6px;
			min-height: 26px;
		}
		QComboBox:focus {
			border: 1px solid #4a90d9;
		}
		QComboBox QAbstractItemView {
			border: 1px solid #d0d0d0;
			outline: none;
		}
		QComboBox QAbstractItemView::item {
			min-height: 26px;
			padding: 0 8px;
		}
	)");
	basicLayout->addRow(partTypeLabel, m_partTypeCombo);

	QLabel* electrodeTypeLabel = new QLabel("电极类型：");
	electrodeTypeLabel->setStyleSheet("border: none; background: transparent;");
	m_electrodeTypeCombo = new QComboBox(this);
	m_electrodeTypeCombo->setStyleSheet(R"(
		QComboBox {
			background-color: #ffffff;
			border: 1px solid #d0d0d0;
			border-radius: 3px;
			padding: 2px 6px;
			min-height: 26px;
		}
		QComboBox:focus {
			border: 1px solid #4a90d9;
		}
		QComboBox QAbstractItemView {
			border: 1px solid #d0d0d0;
			outline: none;
		}
		QComboBox QAbstractItemView::item {
			min-height: 26px;
			padding: 0 8px;
		}
	)");
	basicLayout->addRow(electrodeTypeLabel, m_electrodeTypeCombo);

	connect(m_partTypeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onPartTypeChanged(int)));
		connect(m_machineTypeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onMachineTypeChanged(int)));
		connect(m_machineNameCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onMachineNameChanged(int)));

		leftLayout->addLayout(basicLayout);
	leftLayout->addStretch();

	// 右侧 - 坐标参数
	QVBoxLayout* rightLayout = new QVBoxLayout();
	rightLayout->setSpacing(8);

	QLabel* coordLabel = new QLabel("坐标参数", this);
	coordLabel->setFont(labelFont);
	rightLayout->addWidget(coordLabel);

	QGridLayout* coordGrid = new QGridLayout();
	coordGrid->setSpacing(12);
	coordGrid->setColumnStretch(0, 1);
	coordGrid->setColumnStretch(1, 1);

	auto createAxisGroup = [this, &coordGrid](const QString& title,
		QDoubleSpinBox*& xSpin, QDoubleSpinBox*& ySpin, QDoubleSpinBox*& zSpin, int row, int col) {
		QGroupBox* group = new QGroupBox(title, this);
		QFormLayout* layout = new QFormLayout(group);
		layout->setVerticalSpacing(8);
		layout->setHorizontalSpacing(10);
		layout->setContentsMargins(12, 20, 12, 10);
		layout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

		xSpin = new QDoubleSpinBox(this);
		xSpin->setRange(-9999.999, 9999.999);
		xSpin->setDecimals(3);
		xSpin->setValue(0.0);
		xSpin->setMinimumWidth(110);

		ySpin = new QDoubleSpinBox(this);
		ySpin->setRange(-9999.999, 9999.999);
		ySpin->setDecimals(3);
		ySpin->setValue(0.0);
		ySpin->setMinimumWidth(110);

		zSpin = new QDoubleSpinBox(this);
		zSpin->setRange(-9999.999, 9999.999);
		zSpin->setDecimals(3);
		zSpin->setValue(0.0);
		zSpin->setMinimumWidth(110);

		layout->addRow("X:", xSpin);
		layout->addRow("Y:", ySpin);
		layout->addRow("Z:", zSpin);

		coordGrid->addWidget(group, row, col);
	};

	createAxisGroup("U轴中心", m_uAxisCenterXSpin, m_uAxisCenterYSpin, m_uAxisCenterZSpin, 0, 0);
	createAxisGroup("V轴中心", m_vAxisCenterXSpin, m_vAxisCenterYSpin, m_vAxisCenterZSpin, 0, 1);
	createAxisGroup("W轴中心", m_wAxisCenterXSpin, m_wAxisCenterYSpin, m_wAxisCenterZSpin, 1, 0);
	createAxisGroup("上夹具中心", m_upChuckCenterXSpin, m_upChuckCenterYSpin, m_upChuckCenterZSpin, 1, 1);
	createAxisGroup("下夹具中心", m_downChuckCenterXSpin, m_downChuckCenterYSpin, m_downChuckCenterZSpin, 2, 0);

	rightLayout->addLayout(coordGrid);
	rightLayout->addStretch();

	mainCardLayout->addLayout(leftLayout, 2);
	mainCardLayout->addLayout(rightLayout, 3);

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

	if (m_partTypeCombo->count() > 0) {
		loadElectrodesForPart(m_partTypeCombo->itemText(0));
	}
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
	QString partType = m_partTypeCombo->currentText().trimmed();
	QString electrodeType = m_electrodeTypeCombo->currentText().trimmed();

	if (partRfid.isEmpty()) {
		m_lastError = "请输入工件RFID";
		return false;
	}

	if (electrodeRfid.isEmpty()) {
		m_lastError = "请输入电极RFID";
		return false;
	}

	if (partType.isEmpty()) {
		m_lastError = "请选择工件类型";
		return false;
	}

	if (electrodeType.isEmpty() || electrodeType == "无可用电极") {
		m_lastError = "请选择电极类型";
		return false;
	}

	QJsonObject edmParams;

	QJsonArray uAxisCenter;
	uAxisCenter.append(m_uAxisCenterXSpin->value());
	uAxisCenter.append(m_uAxisCenterYSpin->value());
	uAxisCenter.append(m_uAxisCenterZSpin->value());
	edmParams["UAxisCenter"] = uAxisCenter;

	QJsonArray vAxisCenter;
	vAxisCenter.append(m_vAxisCenterXSpin->value());
	vAxisCenter.append(m_vAxisCenterYSpin->value());
	vAxisCenter.append(m_vAxisCenterZSpin->value());
	edmParams["VAxisCenter"] = vAxisCenter;

	QJsonArray wAxisCenter;
	wAxisCenter.append(m_wAxisCenterXSpin->value());
	wAxisCenter.append(m_wAxisCenterYSpin->value());
	wAxisCenter.append(m_wAxisCenterZSpin->value());
	edmParams["WAxisCenter"] = wAxisCenter;

	QJsonArray upChuck;
	upChuck.append(m_upChuckCenterXSpin->value());
	upChuck.append(m_upChuckCenterYSpin->value());
	upChuck.append(m_upChuckCenterZSpin->value());
	edmParams["UpChuck"] = upChuck;

	QJsonArray downChuck;
	downChuck.append(m_downChuckCenterXSpin->value());
	downChuck.append(m_downChuckCenterYSpin->value());
	downChuck.append(m_downChuckCenterZSpin->value());
	edmParams["DownChuck"] = downChuck;

	QJsonArray electrodePos;
	
	QString posName = electrodeType;
	
	QString appDir = QCoreApplication::applicationDirPath();
	QString configFile = appDir + "/PartConfig/" + partType + ".json";
	QFile file(configFile);
	if (file.exists() && file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QByteArray data = file.readAll();
		file.close();

		QJsonParseError error;
		QJsonDocument doc = QJsonDocument::fromJson(data, &error);
		if (error.error == QJsonParseError::NoError && doc.isObject()) {
			QJsonObject obj = doc.object();
			if (obj.contains("holePositions") && obj["holePositions"].isArray()) {
				QJsonArray holePositions = obj["holePositions"].toArray();
				for (const QJsonValue& val : holePositions) {
					QJsonObject holePos = val.toObject();
					if (holePos.contains("name") && holePos["name"].toString() == electrodeType) {
						if (holePos.contains("id")) {
							posName = holePos["id"].toString();
						}
						break;
					}
				}
			}
		}
	}

	QJsonObject posObj;
	posObj["PosName"] = posName;
	
	QJsonArray beginArray;
	beginArray.append(0.0);
	beginArray.append(0.0);
	beginArray.append(0.0);
	beginArray.append(0.0);
	beginArray.append(0.0);
	beginArray.append(0.0);
	posObj["Begin"] = beginArray;

	QJsonArray endArray;
	endArray.append(0.0);
	endArray.append(0.0);
	endArray.append(0.0);
	endArray.append(0.0);
	endArray.append(0.0);
	endArray.append(0.0);
	posObj["End"] = endArray;

	electrodePos.append(posObj);

	return m_pointCloudService->executeSparkMachineProgram(
		machineType,
		partType,
		electrodeType,
		partRfid,
		electrodeRfid,
		electrodePos,
		edmParams,
		m_lastError
	);
}

void SparkMachineProgramDialog::onOperationCompleted(bool success)
{
	if (success) {
		saveConfigToFile();
		setProgressText("✅ 火花机程序创建完成");
	}
	else {
		setProgressText(QString("❌ 火花机程序创建失败：%1").arg(m_lastError));
	}
}

void SparkMachineProgramDialog::loadConfig()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString configFile = appDir + "/config/SparkMachineProgramConfig.json";

	QFile file(configFile);
	if (!file.exists()) {
		return;
	}

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		return;
	}

	QByteArray data = file.readAll();
	file.close();

	QJsonParseError error;
	QJsonDocument doc = QJsonDocument::fromJson(data, &error);
	if (error.error != QJsonParseError::NoError || !doc.isObject()) {
		return;
	}

	QJsonObject obj = doc.object();

	if (obj.contains("machineType")) {
		int index = m_machineTypeCombo->findData(obj["machineType"].toString());
		if (index >= 0) {
			m_machineTypeCombo->setCurrentIndex(index);
		}
	}

	updateMachineNames();

	if (obj.contains("machineName")) {
		QString machineName = obj["machineName"].toString();
		int index = m_machineNameCombo->findText(machineName);
		if (index >= 0) {
			m_machineNameCombo->setCurrentIndex(index);
		}
	}

	if (obj.contains("partRfid")) {
		m_partRfidEdit->setText(obj["partRfid"].toString());
	}

	if (obj.contains("electrodeRfid")) {
		m_electrodeRfidEdit->setText(obj["electrodeRfid"].toString());
	}

	if (obj.contains("partType")) {
		QString partType = obj["partType"].toString();
		int index = m_partTypeCombo->findText(partType);
		if (index >= 0) {
			m_partTypeCombo->setCurrentIndex(index);
		}
	}

	if (m_partTypeCombo->count() > 0) {
		QString currentPartType = m_partTypeCombo->currentText();
		if (!currentPartType.isEmpty()) {
			loadElectrodesForPart(currentPartType);
			
			if (obj.contains("electrodeType")) {
				QString electrodeType = obj["electrodeType"].toString();
				int index = m_electrodeTypeCombo->findText(electrodeType);
				if (index >= 0) {
					m_electrodeTypeCombo->setCurrentIndex(index);
				}
			}
		}
	}

	QString currentMachineName = m_machineNameCombo->currentText();
	QJsonObject machineParams;

	if (obj.contains("machines") && obj["machines"].isObject()) {
		QJsonObject machinesObj = obj["machines"].toObject();
		if (machinesObj.contains(currentMachineName)) {
			machineParams = machinesObj[currentMachineName].toObject();
		}
	} else {
		machineParams = obj;
	}

	if (machineParams.contains("UAxisCenter") && machineParams["UAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["UAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_uAxisCenterXSpin->setValue(arr[0].toDouble());
			m_uAxisCenterYSpin->setValue(arr[1].toDouble());
			m_uAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("VAxisCenter") && machineParams["VAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["VAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_vAxisCenterXSpin->setValue(arr[0].toDouble());
			m_vAxisCenterYSpin->setValue(arr[1].toDouble());
			m_vAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("WAxisCenter") && machineParams["WAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["WAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_wAxisCenterXSpin->setValue(arr[0].toDouble());
			m_wAxisCenterYSpin->setValue(arr[1].toDouble());
			m_wAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("UpChuck") && machineParams["UpChuck"].isArray()) {
		QJsonArray arr = machineParams["UpChuck"].toArray();
		if (arr.size() >= 3) {
			m_upChuckCenterXSpin->setValue(arr[0].toDouble());
			m_upChuckCenterYSpin->setValue(arr[1].toDouble());
			m_upChuckCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("DownChuck") && machineParams["DownChuck"].isArray()) {
		QJsonArray arr = machineParams["DownChuck"].toArray();
		if (arr.size() >= 3) {
			m_downChuckCenterXSpin->setValue(arr[0].toDouble());
			m_downChuckCenterYSpin->setValue(arr[1].toDouble());
			m_downChuckCenterZSpin->setValue(arr[2].toDouble());
		}
	}
}

void SparkMachineProgramDialog::saveConfigToFile()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString configDir = appDir + "/config";
	QDir dir(configDir);
	if (!dir.exists()) {
		dir.mkpath(configDir);
	}
	QString configFile = configDir + "/SparkMachineProgramConfig.json";

	QJsonObject obj;

	QFile existingFile(configFile);
	if (existingFile.exists() && existingFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QByteArray data = existingFile.readAll();
		existingFile.close();
		QJsonParseError error;
		QJsonDocument doc = QJsonDocument::fromJson(data, &error);
		if (error.error == QJsonParseError::NoError && doc.isObject()) {
			obj = doc.object();
		}
	}

	obj["machineType"] = m_machineTypeCombo->currentData().toString();
	obj["machineName"] = m_machineNameCombo->currentText();
	obj["partRfid"] = m_partRfidEdit->text();
	obj["electrodeRfid"] = m_electrodeRfidEdit->text();
	obj["partType"] = m_partTypeCombo->currentText();
	obj["electrodeType"] = m_electrodeTypeCombo->currentText();

	QJsonObject machinesObj;
	if (obj.contains("machines") && obj["machines"].isObject()) {
		machinesObj = obj["machines"].toObject();
	}

	QJsonObject currentMachineParams;

	QJsonArray uAxisCenter;
	uAxisCenter.append(m_uAxisCenterXSpin->value());
	uAxisCenter.append(m_uAxisCenterYSpin->value());
	uAxisCenter.append(m_uAxisCenterZSpin->value());
	currentMachineParams["UAxisCenter"] = uAxisCenter;

	QJsonArray vAxisCenter;
	vAxisCenter.append(m_vAxisCenterXSpin->value());
	vAxisCenter.append(m_vAxisCenterYSpin->value());
	vAxisCenter.append(m_vAxisCenterZSpin->value());
	currentMachineParams["VAxisCenter"] = vAxisCenter;

	QJsonArray wAxisCenter;
	wAxisCenter.append(m_wAxisCenterXSpin->value());
	wAxisCenter.append(m_wAxisCenterYSpin->value());
	wAxisCenter.append(m_wAxisCenterZSpin->value());
	currentMachineParams["WAxisCenter"] = wAxisCenter;

	QJsonArray upChuck;
	upChuck.append(m_upChuckCenterXSpin->value());
	upChuck.append(m_upChuckCenterYSpin->value());
	upChuck.append(m_upChuckCenterZSpin->value());
	currentMachineParams["UpChuck"] = upChuck;

	QJsonArray downChuck;
	downChuck.append(m_downChuckCenterXSpin->value());
	downChuck.append(m_downChuckCenterYSpin->value());
	downChuck.append(m_downChuckCenterZSpin->value());
	currentMachineParams["DownChuck"] = downChuck;

	machinesObj[m_machineNameCombo->currentText()] = currentMachineParams;
	obj["machines"] = machinesObj;

	QJsonDocument doc(obj);
	QFile file(configFile);
	if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		file.write(doc.toJson(QJsonDocument::Indented));
		file.close();
	}
}

void SparkMachineProgramDialog::loadPartTypes()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString configDir = appDir + "/PartConfig";
	QDir dir(configDir);

	m_partElectrodeMap.clear();
	m_partTypeCombo->clear();

	if (!dir.exists()) {
		return;
	}

	QStringList filters;
	filters << "*.json";
	dir.setNameFilters(filters);

	QStringList files = dir.entryList(filters, QDir::Files);
	if (files.isEmpty()) {
		return;
	}

	for (const QString& file : files) {
		QString partName = file.left(file.lastIndexOf('.'));
		QString filePath = dir.filePath(file);

		QFile jsonFile(filePath);
		if (!jsonFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
			continue;
		}

		QByteArray data = jsonFile.readAll();
		QJsonParseError error;
		QJsonDocument doc = QJsonDocument::fromJson(data, &error);

		if (error.error != QJsonParseError::NoError) {
			continue;
		}

		QStringList electrodes;
		QJsonArray electrodesArray = doc["holePositions"].toArray();
		for (const QJsonValue& val : electrodesArray) {
			QJsonObject obj = val.toObject();
			electrodes.append(obj["id"].toString());
		}

		m_partTypeCombo->addItem(partName);
		m_partElectrodeMap[partName] = electrodes;
	}
}

void SparkMachineProgramDialog::loadElectrodesForPart(const QString& partName)
{
	m_electrodeTypeCombo->clear();

	if (m_partElectrodeMap.contains(partName)) {
		QStringList electrodes = m_partElectrodeMap[partName];
		if (electrodes.isEmpty()) {
			m_electrodeTypeCombo->addItem("无可用电极");
			m_electrodeTypeCombo->setEnabled(false);
		} else {
			m_electrodeTypeCombo->addItems(electrodes);
			m_electrodeTypeCombo->setEnabled(true);
		}
	} else {
		m_electrodeTypeCombo->addItem("无可用电极");
		m_electrodeTypeCombo->setEnabled(false);
	}
}

void SparkMachineProgramDialog::onPartTypeChanged(int index)
{
	QString partName = m_partTypeCombo->itemText(index);
	loadElectrodesForPart(partName);
}

void SparkMachineProgramDialog::updateUIState()
{
	MachineStatusDialog::updateUIState();
}

void SparkMachineProgramDialog::updateMachineNames()
{
	m_machineNameCombo->clear();
	QString machineType = m_machineTypeCombo->currentData().toString();
	if (m_machineNameMap.contains(machineType)) {
		m_machineNameCombo->addItems(m_machineNameMap[machineType]);
	}
}

void SparkMachineProgramDialog::onMachineTypeChanged(int index)
{
	Q_UNUSED(index);
	updateMachineNames();
}

void SparkMachineProgramDialog::onMachineNameChanged(int index)
{
	Q_UNUSED(index);
	loadMachineParams();
}

void SparkMachineProgramDialog::loadMachineParams()
{
	m_uAxisCenterXSpin->setValue(0.0);
	m_uAxisCenterYSpin->setValue(0.0);
	m_uAxisCenterZSpin->setValue(0.0);
	m_vAxisCenterXSpin->setValue(0.0);
	m_vAxisCenterYSpin->setValue(0.0);
	m_vAxisCenterZSpin->setValue(0.0);
	m_wAxisCenterXSpin->setValue(0.0);
	m_wAxisCenterYSpin->setValue(0.0);
	m_wAxisCenterZSpin->setValue(0.0);
	m_upChuckCenterXSpin->setValue(0.0);
	m_upChuckCenterYSpin->setValue(0.0);
	m_upChuckCenterZSpin->setValue(0.0);
	m_downChuckCenterXSpin->setValue(0.0);
	m_downChuckCenterYSpin->setValue(0.0);
	m_downChuckCenterZSpin->setValue(0.0);

	QString appDir = QCoreApplication::applicationDirPath();
	QString configFile = appDir + "/config/SparkMachineProgramConfig.json";

	QFile file(configFile);
	if (!file.exists() || !file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		return;
	}

	QByteArray data = file.readAll();
	file.close();

	QJsonParseError error;
	QJsonDocument doc = QJsonDocument::fromJson(data, &error);
	if (error.error != QJsonParseError::NoError || !doc.isObject()) {
		return;
	}

	QJsonObject obj = doc.object();
	if (!obj.contains("machines") || !obj["machines"].isObject()) {
		return;
	}

	QJsonObject machinesObj = obj["machines"].toObject();
	QString currentMachineName = m_machineNameCombo->currentText();
	if (!machinesObj.contains(currentMachineName)) {
		return;
	}

	QJsonObject machineParams = machinesObj[currentMachineName].toObject();

	if (machineParams.contains("UAxisCenter") && machineParams["UAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["UAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_uAxisCenterXSpin->setValue(arr[0].toDouble());
			m_uAxisCenterYSpin->setValue(arr[1].toDouble());
			m_uAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("VAxisCenter") && machineParams["VAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["VAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_vAxisCenterXSpin->setValue(arr[0].toDouble());
			m_vAxisCenterYSpin->setValue(arr[1].toDouble());
			m_vAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("WAxisCenter") && machineParams["WAxisCenter"].isArray()) {
		QJsonArray arr = machineParams["WAxisCenter"].toArray();
		if (arr.size() >= 3) {
			m_wAxisCenterXSpin->setValue(arr[0].toDouble());
			m_wAxisCenterYSpin->setValue(arr[1].toDouble());
			m_wAxisCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("UpChuck") && machineParams["UpChuck"].isArray()) {
		QJsonArray arr = machineParams["UpChuck"].toArray();
		if (arr.size() >= 3) {
			m_upChuckCenterXSpin->setValue(arr[0].toDouble());
			m_upChuckCenterYSpin->setValue(arr[1].toDouble());
			m_upChuckCenterZSpin->setValue(arr[2].toDouble());
		}
	}

	if (machineParams.contains("DownChuck") && machineParams["DownChuck"].isArray()) {
		QJsonArray arr = machineParams["DownChuck"].toArray();
		if (arr.size() >= 3) {
			m_downChuckCenterXSpin->setValue(arr[0].toDouble());
			m_downChuckCenterYSpin->setValue(arr[1].toDouble());
			m_downChuckCenterZSpin->setValue(arr[2].toDouble());
		}
	}
}
