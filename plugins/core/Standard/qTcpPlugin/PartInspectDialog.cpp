#include "PartInspectDialog.h"
#include "PointCloudService.h"
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QCoreApplication>
#include <QDir>
#include <QFile>

PartInspectDialog::PartInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("工件检测");
	setFixedSize(450, 320);
	init();
}

PartInspectDialog::~PartInspectDialog()
{
}

void PartInspectDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(20, 16, 20, 16);
	m_mainLayout->setSpacing(10);

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
    )");

	QFormLayout* formLayout = new QFormLayout(formCard);
	formLayout->setContentsMargins(16, 14, 16, 14);
	formLayout->setVerticalSpacing(12);
	formLayout->setHorizontalSpacing(16);
	formLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

	QLabel* typeLabel = new QLabel("工件类型：");
	typeLabel->setStyleSheet("border: none; background: transparent;");
	m_partTypeCombo = new QComboBox(this);
	loadPartTypes();
	formLayout->addRow(typeLabel, m_partTypeCombo);

	QLabel* rfidLabel = new QLabel("RFID：");
	rfidLabel->setStyleSheet("border: none; background: transparent;");
	m_rfidEdit = new QLineEdit(this);
	m_rfidEdit->setPlaceholderText("请输入RFID编号");
	formLayout->addRow(rfidLabel, m_rfidEdit);

	m_mainLayout->addWidget(formCard);

	// --- 分隔线 ---
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

void PartInspectDialog::loadPartTypes()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString configDir = appDir + "/PartConfig";
	QDir dir(configDir);

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
		m_partTypeCombo->addItem(partName);
	}
}
