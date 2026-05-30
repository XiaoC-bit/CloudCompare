#include "ElectrodeInspectDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>
#include <QFrame>

ElectrodeInspectDialog::ElectrodeInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("电极检测");
	setFixedSize(750, 600);
	init();
}

ElectrodeInspectDialog::~ElectrodeInspectDialog()
{
}

void ElectrodeInspectDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(20, 16, 20, 16);
	m_mainLayout->setSpacing(10);

	m_infoFrame = new QFrame(this);
	m_infoFrame->setFrameShape(QFrame::StyledPanel);
	m_infoFrame->setMinimumHeight(170);
	m_infoFrame->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
	m_infoFrame->setStyleSheet("QFrame { background-color: #f5f7fa; border: 1px solid #d0d7e0; border-radius: 8px; }");
	QHBoxLayout* infoLayout = new QHBoxLayout(m_infoFrame);
	infoLayout->setContentsMargins(16, 16, 16, 16);
	infoLayout->setSpacing(20);

	m_schematicLabel = new QLabel(this);
	m_schematicLabel->setFixedSize(280, 140);
	m_schematicLabel->setStyleSheet("QLabel { background-color: white; border: 2px solid #c0c8d0; border-radius: 6px; }");
	m_schematicLabel->setAlignment(Qt::AlignCenter);
	m_schematicLabel->setScaledContents(true);

	QPixmap schematicPixmap(":/CC/plugin/qTcpPlugin/res/elec_op.png");
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
		"<p style='margin: 4px 0;'>1. 选择正确的<strong>工件类型</strong></p>"
		"<p style='margin: 4px 0;'>2. 选择对应的<strong>电极类型</strong></p>"
		"<p style='margin: 4px 0;'>3. 输入对应<strong>RFID编号</strong></p>"
		"<p style='margin: 4px 0;'>4. 确认电极已<strong>正确安装</strong></p>"
		"<p style='margin: 10px 0 0 0; color: #cc6600; font-weight: bold;'>⚠️ 确认以上步骤完成后，再点击「开始检测」</p>"
	);

	infoLayout->addWidget(m_instructionLabel, 1);

	m_mainLayout->addWidget(m_infoFrame);

	// --- 标题 ---
	QLabel* headerLabel = new QLabel("检测参数", this);
	QFont   font        = headerLabel->font();
	font.setBold(true);
	font.setPointSize(font.pointSize() + 1);
	headerLabel->setFont(font);
	m_mainLayout->addWidget(headerLabel);

	// --- 表单容器 ---
	QWidget* formCard = new QWidget(this);
	formCard->setStyleSheet(R"(
    QWidget {
        background-color: #f7f7f9;
        border: 1px solid #d0d0d0;
        border-radius: 4px;
    }
    QComboBox, QLineEdit {
        background-color: #ffffff;
        border: 1px solid #d0d0d0;
        border-radius: 3px;
        padding: 2px 6px;
        min-height: 26px;
    }
    QComboBox:focus, QLineEdit:focus {
        border: 1px solid #4a90d9;
    }
    QComboBox QAbstractItemView {
        border: 1px solid #d0d0d0;
        outline: none;
    }
    QComboBox QAbstractItemView::item {
        min-height: 58px;
        padding: 0 8px;
    }
    QComboBox QAbstractItemView::item:selected {
        background-color: #dce8ff;
        color: #000000;
    }
)");

	QFormLayout* formLayout = new QFormLayout(formCard);
	formLayout->setContentsMargins(16, 14, 16, 14);
	formLayout->setVerticalSpacing(12);
	formLayout->setHorizontalSpacing(16);
	formLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

	QLabel* partTypeLabel = new QLabel("工件类型：");
	partTypeLabel->setStyleSheet("border: none; background: transparent;");
	m_partTypeCombo = new QComboBox(this);
	loadPartTypes();
	formLayout->addRow(partTypeLabel, m_partTypeCombo);

	QLabel* typeLabel = new QLabel("电极类型：");
	typeLabel->setStyleSheet("border: none; background: transparent;");
	m_electrodeTypeCombo = new QComboBox(this);
	formLayout->addRow(typeLabel, m_electrodeTypeCombo);

	connect(m_partTypeCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onPartTypeChanged(int)));

	QLabel* rfidLabel = new QLabel("RFID：");
	rfidLabel->setStyleSheet("border: none; background: transparent;");
	m_rfidEdit = new QLineEdit(this);
	m_rfidEdit->setPlaceholderText("请输入RFID编号");
	formLayout->addRow(rfidLabel, m_rfidEdit);

	m_mainLayout->addWidget(formCard);

	// --- 弹簧 + 分隔线 ---
	m_mainLayout->addStretch();

	QFrame* separator = new QFrame(this);
	separator->setFrameShape(QFrame::HLine);
	separator->setFrameShadow(QFrame::Sunken);
	m_mainLayout->addWidget(separator);

	// --- 底部按钮 ---
	m_buttonLayout = new QHBoxLayout();
	m_buttonLayout->setSpacing(8);
	m_buttonLayout->addStretch();

	m_cancelButton = new QPushButton("取消", this);
	m_cancelButton->setFixedWidth(80);
	m_cancelButton->setFixedHeight(32);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
	m_buttonLayout->addWidget(m_cancelButton);

	m_startButton = new QPushButton("开始检测", this);
	m_startButton->setFixedWidth(100);
	m_startButton->setFixedHeight(32);
	m_startButton->setDefault(true);
	connect(m_startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);
	m_buttonLayout->addWidget(m_startButton);

	m_mainLayout->addLayout(m_buttonLayout);

	if (m_partTypeCombo->count() > 0) {
		loadElectrodesForPart(m_partTypeCombo->itemText(0));
	}
}

void ElectrodeInspectDialog::onOperationStarted()
{
	setProgressText("开始电极检测...");
}

bool ElectrodeInspectDialog::performOperation()
{
	
	QString partType = m_partTypeCombo->currentText();
	QString electrodeType = m_electrodeTypeCombo->currentText();
	QString rfid = m_rfidEdit->text();

	if (rfid.isEmpty()) {
		setProgressText("❌ 请输入RFID");
		return false;
	}

	return m_pointCloudService->executeElectrodeInspect(partType,electrodeType, rfid);
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

void ElectrodeInspectDialog::loadPartTypes()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString configDir = appDir + "/PartConfig";
	QDir dir(configDir);

	m_partElectrodeMap.clear();

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
		QJsonArray electrodesArray = doc["electrodes"].toArray();
		for (const QJsonValue& val : electrodesArray) {
			QJsonObject obj = val.toObject();
			electrodes.append(obj["electrodeName"].toString());
		}

		m_partTypeCombo->addItem(partName);
		m_partElectrodeMap[partName] = electrodes;
	}
}

void ElectrodeInspectDialog::loadElectrodesForPart(const QString& partName)
{
	m_electrodeTypeCombo->clear();

	if (m_partElectrodeMap.contains(partName)) {
		QStringList electrodes = m_partElectrodeMap[partName];
		if (electrodes.isEmpty()) {
			m_electrodeTypeCombo->addItem("无可用电极");
			m_electrodeTypeCombo->setEnabled(false);
			m_startButton->setEnabled(false);
		} else {
			m_electrodeTypeCombo->addItems(electrodes);
			m_electrodeTypeCombo->setEnabled(true);
			m_startButton->setEnabled(true);
		}
	} else {
		m_electrodeTypeCombo->addItem("无可用电极");
		m_electrodeTypeCombo->setEnabled(false);
		m_startButton->setEnabled(false);
	}
}

void ElectrodeInspectDialog::onPartTypeChanged(int index)
{
	QString partName = m_partTypeCombo->itemText(index);
	loadElectrodesForPart(partName);
}

void ElectrodeInspectDialog::updateUIState()
{
	MachineStatusDialog::updateUIState();

	if (m_electrodeTypeCombo && m_startButton) {
		bool hasElectrode = m_electrodeTypeCombo->count() > 0 && 
							m_electrodeTypeCombo->itemText(0) != "无可用电极";
		if (!hasElectrode) {
			m_startButton->setEnabled(false);
		}
	}
}
