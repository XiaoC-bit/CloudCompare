#include "CalibrationDialog.h"
#include "PointCloudService.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <ccMainAppInterface.h>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QCoreApplication>
#include <qevent.h>
#include <QLabel>
#include <QFrame>
#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>

QVector<CalibrationDialog::Position> CalibrationDialog::loadDefaultPositions()
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString calibrationPosFile = appDir + "/Template/CalibrationPos.json";

    QFile file(calibrationPosFile);
    if (!file.exists()) {
        return defaultHardcodedPositions();
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return defaultHardcodedPositions();
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return defaultHardcodedPositions();
    }

    QJsonObject obj = doc.object();
    if (!obj.contains("positions") || !obj["positions"].isArray()) {
        return defaultHardcodedPositions();
    }

    QJsonArray positionsArray = obj["positions"].toArray();
    if (positionsArray.isEmpty()) {
        return defaultHardcodedPositions();
    }

    QVector<Position> positions;
    positions.reserve(positionsArray.size());

    for (int i = 0; i < positionsArray.size(); ++i) {
        const QJsonValue entry = positionsArray.at(i);
        if (entry.isObject()) {
            const QJsonObject posObj = entry.toObject();
            if (!posObj.contains("x") || !posObj.contains("y") || !posObj.contains("z")) {
                return defaultHardcodedPositions();
            }
            positions.push_back(Position(
                posObj["x"].toDouble(),
                posObj["y"].toDouble(),
                posObj["z"].toDouble()
            ));
        } else if (entry.isArray()) {
            const QJsonArray posArray = entry.toArray();
            if (posArray.size() != 3) {
                return defaultHardcodedPositions();
            }
            positions.push_back(Position(
                posArray[0].toDouble(),
                posArray[1].toDouble(),
                posArray[2].toDouble()
            ));
        } else {
            return defaultHardcodedPositions();
        }
    }

    return positions;
}

QVector<CalibrationDialog::Position> CalibrationDialog::defaultHardcodedPositions()
{
    QVector<Position> positions;
    positions.push_back(Position(0, 0, 0));
    positions.push_back(Position(5, 0, 0));
    positions.push_back(Position(5, 5, 0));
    positions.push_back(Position(5, 10, 0));
    positions.push_back(Position(10, 10, 0));
    positions.push_back(Position(0, 0, -2.5));
    positions.push_back(Position(0, 0, -5));
    return positions;
}

CalibrationDialog::CalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : MachineStatusDialog(app, pointCloudService, parent)
    , m_positions(loadDefaultPositions())
{
	m_defaultPositionCount = m_positions.size();
	setWindowTitle("激光相机标定");
	setFixedSize(450, 600);
	init();
}

CalibrationDialog::~CalibrationDialog()
{
}

void CalibrationDialog::setupAdditionalUI()
{
	m_mainLayout->setContentsMargins(20, 16, 20, 16);
	m_mainLayout->setSpacing(10);

	// --- 标题行 + 新增按钮 ---
	QHBoxLayout* headerLayout = new QHBoxLayout();
	headerLayout->setContentsMargins(0, 0, 0, 0);

	QLabel* headerLabel = new QLabel("标定位置", this);
	QFont   font        = headerLabel->font();
	font.setBold(true);
	font.setPointSize(font.pointSize() + 1);
	headerLabel->setFont(font);
	headerLayout->addWidget(headerLabel);
	headerLayout->addStretch();

	m_addButton = new QPushButton("＋ 新增", this);
	m_addButton->setFixedHeight(28);
	m_addButton->setFixedWidth(72);
	connect(m_addButton, &QPushButton::clicked, this, &CalibrationDialog::onAddPosition);
	headerLayout->addWidget(m_addButton);

	m_generateButton = new QPushButton("生成位置", this);
	m_generateButton->setFixedHeight(28);
	m_generateButton->setFixedWidth(80);
	connect(m_generateButton, &QPushButton::clicked, this, &CalibrationDialog::onGeneratePositions);
	headerLayout->addWidget(m_generateButton);

	m_mainLayout->addLayout(headerLayout);

	// --- 表格 ---
	m_tableWidget = new QTableWidget(this);
	m_tableWidget->setColumnCount(5);
	m_tableWidget->setHorizontalHeaderLabels({"X (mm)", "Y (mm)", "Z (mm)", "读取", "操作"});
	m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_tableWidget->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
	m_tableWidget->setColumnWidth(3, 72);
	m_tableWidget->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
	m_tableWidget->setColumnWidth(4, 64);
	m_tableWidget->setShowGrid(false);
	m_tableWidget->setAlternatingRowColors(true);
	m_tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	m_tableWidget->verticalHeader()->setVisible(false);
	m_tableWidget->horizontalHeader()->setHighlightSections(false);
	m_tableWidget->verticalHeader()->setDefaultSectionSize(36); // 固定行高
	m_tableWidget->setMinimumHeight(220);

	// stylesheet 统一风格
	m_tableWidget->setStyleSheet(R"(
        QTableWidget {
            border: 1px solid #d0d0d0;
            border-radius: 4px;
            background-color: #ffffff;
            alternate-background-color: #f7f7f9;
            outline: none;
        }
        QHeaderView::section {
            background-color: #f0f0f2;
            color: #444444;
            font-weight: bold;
            padding: 4px 8px;
            border: none;
            border-bottom: 1px solid #d0d0d0;
        }
        QTableWidget::item {
            padding: 0px 4px;
        }
        QTableWidget::item:selected {
            background-color: #dce8ff;
            color: #000000;
        }
    )");

	m_mainLayout->addWidget(m_tableWidget, 1); // stretch factor=1，表格优先撑高

	// --- 分隔线 ---
	QFrame* separator = new QFrame(this);
	separator->setFrameShape(QFrame::HLine);
	separator->setFrameShadow(QFrame::Sunken);
	m_mainLayout->addWidget(separator);

	// --- 保存提示 ---
	QLabel* saveTipLabel = new QLabel("💡 保存后，自动化流程将使用当前标定位置", this);
	saveTipLabel->setStyleSheet("color: #666666; font-size: 12px;");
	m_mainLayout->addWidget(saveTipLabel);

	// --- 底部按钮 ---
	m_buttonLayout = new QHBoxLayout();
	m_buttonLayout->setSpacing(8);

	m_resetButton = new QPushButton("复位", this);
	m_resetButton->setFixedWidth(80);
	m_resetButton->setFixedHeight(32);
	connect(m_resetButton, &QPushButton::clicked, this, &CalibrationDialog::onReset);
	m_buttonLayout->addWidget(m_resetButton);

	m_saveButton = new QPushButton("保存", this);
	m_saveButton->setFixedWidth(80);
	m_saveButton->setFixedHeight(32);
	connect(m_saveButton, &QPushButton::clicked, this, &CalibrationDialog::onSavePositions);
	m_buttonLayout->addWidget(m_saveButton);

	m_buttonLayout->addStretch();

	m_cancelButton = new QPushButton("取消", this);
	m_cancelButton->setFixedWidth(80);
	m_cancelButton->setFixedHeight(32);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
	m_buttonLayout->addWidget(m_cancelButton);

	m_startButton = new QPushButton("开始标定", this);
	m_startButton->setFixedWidth(100);
	m_startButton->setFixedHeight(32);
	m_startButton->setDefault(true);
	connect(m_startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);
	m_buttonLayout->addWidget(m_startButton);

	m_mainLayout->addLayout(m_buttonLayout);

	populateTable();
}

void CalibrationDialog::populateTable()
{
	m_tableWidget->setUpdatesEnabled(false); // 批量更新，避免闪烁
	m_tableWidget->setRowCount(m_positions.size());

	for (int i = 0; i < m_positions.size(); ++i)
	{
		const Position& pos = m_positions[i];

		auto makeSpinBox = [this, i](double value, int col) -> QDoubleSpinBox*
		{
			QDoubleSpinBox* sb = new QDoubleSpinBox();
			sb->setMinimum(-1000);
			sb->setMaximum(1000);
			sb->setDecimals(3);
			sb->setValue(value);
			sb->setButtonSymbols(QAbstractSpinBox::NoButtons);
			sb->setFrame(false);
			sb->setAlignment(Qt::AlignCenter);
			sb->setStyleSheet("background: transparent; padding: 0 4px;");
			connect(sb, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this, i, col](double val) {
				switch (col) {
				case 0: m_positions[i].x = val; break;
				case 1: m_positions[i].y = val; break;
				case 2: m_positions[i].z = val; break;
				}
			});
			return sb;
		};

		m_tableWidget->setCellWidget(i, 0, makeSpinBox(pos.x, 0));
		m_tableWidget->setCellWidget(i, 1, makeSpinBox(pos.y, 1));
		m_tableWidget->setCellWidget(i, 2, makeSpinBox(pos.z, 2));

		QPushButton* readButton = new QPushButton("读取");
		readButton->setFixedHeight(24);
		readButton->setProperty("row", i);
		readButton->setStyleSheet(R"(
            QPushButton {
                color: #3366cc;
                border: 1px solid #3366cc;
                border-radius: 3px;
                background: transparent;
                font-size: 12px;
                padding: 0 6px;
            }
            QPushButton:hover {
                background-color: #f0f5ff;
            }
        )");
		connect(readButton, &QPushButton::clicked, this, &CalibrationDialog::onReadFromDevice);

		QWidget*     readCellWidget = new QWidget();
		QHBoxLayout* readCellLayout = new QHBoxLayout(readCellWidget);
		readCellLayout->setContentsMargins(4, 2, 4, 2);
		readCellLayout->addWidget(readButton);
		m_tableWidget->setCellWidget(i, 3, readCellWidget);

		QPushButton* deleteButton = new QPushButton("删除");
		deleteButton->setFixedHeight(24);
		deleteButton->setProperty("row", i);
		deleteButton->setStyleSheet(R"(
            QPushButton {
                color: #cc3333;
                border: 1px solid #cc3333;
                border-radius: 3px;
                background: transparent;
                font-size: 12px;
                padding: 0 6px;
            }
            QPushButton:hover {
                background-color: #fff0f0;
            }
        )");
		connect(deleteButton, &QPushButton::clicked, this, &CalibrationDialog::onDeleteRow);

		QWidget*     cellWidget = new QWidget();
		QHBoxLayout* cellLayout = new QHBoxLayout(cellWidget);
		cellLayout->setContentsMargins(4, 2, 4, 2);
		cellLayout->addWidget(deleteButton);
		m_tableWidget->setCellWidget(i, 4, cellWidget);
	}

	m_tableWidget->setUpdatesEnabled(true);
}

void CalibrationDialog::onAddPosition()
{
	if (m_positions.size() >= 30) {
		QMessageBox::warning(this, "警告", "最多只能添加30组数据");
		return;
	}

	m_positions.append(Position(0, 0, 0));
	populateTable();
}

void CalibrationDialog::onGeneratePositions()
{
	if (m_positions.isEmpty()) {
		QMessageBox::warning(this, "警告", "请先添加或设置第一个标定位置");
		return;
	}

	QDialog dialog(this);
	dialog.setWindowTitle("生成标定位置");
	dialog.setFixedSize(320, 180);

	QVBoxLayout* layout = new QVBoxLayout(&dialog);
	layout->setContentsMargins(20, 20, 20, 20);
	layout->setSpacing(15);

	QHBoxLayout* stepLayout = new QHBoxLayout();
	QLabel* stepLabel = new QLabel("步长 (mm):");
	QDoubleSpinBox* stepSpinBox = new QDoubleSpinBox();
	stepSpinBox->setMinimum(0.1);
	stepSpinBox->setMaximum(50);
	stepSpinBox->setDecimals(1);
	stepSpinBox->setValue(5);
	stepLayout->addWidget(stepLabel);
	stepLayout->addWidget(stepSpinBox);
	layout->addLayout(stepLayout);

	QHBoxLayout* countLayout = new QHBoxLayout();
	QLabel* countLabel = new QLabel("位置个数:");
	QSpinBox* countSpinBox = new QSpinBox();
	countSpinBox->setMinimum(2);
	countSpinBox->setMaximum(30);
	countSpinBox->setValue(7);
	countLayout->addWidget(countLabel);
	countLayout->addWidget(countSpinBox);
	layout->addLayout(countLayout);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	QPushButton* okButton = new QPushButton("确定");
	QPushButton* cancelButton = new QPushButton("取消");
	okButton->setDefault(true);
	buttonLayout->addStretch();
	buttonLayout->addWidget(okButton);
	buttonLayout->addWidget(cancelButton);
	layout->addLayout(buttonLayout);

	connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
	connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

	if (dialog.exec() != QDialog::Accepted) {
		return;
	}

	double step = stepSpinBox->value();
	int count = countSpinBox->value();

	const Position& firstPos = m_positions[0];

	QVector<Position> newPositions;
	newPositions.reserve(count);
	newPositions.append(firstPos);

	if (count >= 2) newPositions.append(Position(firstPos.x + step, firstPos.y, firstPos.z));
	if (count >= 3) newPositions.append(Position(firstPos.x + step, firstPos.y + step, firstPos.z));
	if (count >= 4) newPositions.append(Position(firstPos.x + step, firstPos.y + step * 2, firstPos.z));
	if (count >= 5) newPositions.append(Position(firstPos.x + step * 2, firstPos.y + step * 2, firstPos.z));
	if (count >= 6) newPositions.append(Position(firstPos.x, firstPos.y, firstPos.z - step / 2));
	if (count >= 7) newPositions.append(Position(firstPos.x, firstPos.y, firstPos.z - step));

	for (int i = 7; i < count; ++i) {
		double offset = (i - 6) * step;
		newPositions.append(Position(firstPos.x + offset, firstPos.y, firstPos.z));
	}

	m_positions = newPositions;
	populateTable();

	m_defaultPositionCount = m_positions.size();

	QMessageBox::information(this, "成功", QString("已生成 %1 个标定位置").arg(count));
}

void CalibrationDialog::onReset()
{
	m_positions = loadDefaultPositions();
	populateTable();
}

void CalibrationDialog::onSavePositions()
{
	QMessageBox::StandardButton reply = QMessageBox::question(this, "确认保存",
		"确定要保存当前标定位置吗？\n\n保存后，自动化流程将使用当前标定位置。",
		QMessageBox::Yes | QMessageBox::No);

	if (reply != QMessageBox::Yes) {
		return;
	}

	QString appDir = QCoreApplication::applicationDirPath();
	QString calibrationPosFile = appDir + "/Template/CalibrationPos.json";

	QDir dir(appDir + "/Template");
	if (!dir.exists()) {
		if (!dir.mkpath(appDir + "/Template")) {
			QMessageBox::warning(this, "失败", "无法创建目录: " + appDir + "/Template");
			return;
		}
	}

	QJsonObject obj;
	QJsonArray positionsArray;

	for (const Position& pos : m_positions) {
		QJsonObject posObj;
		posObj["x"] = pos.x;
		posObj["y"] = pos.y;
		posObj["z"] = pos.z;
		positionsArray.append(posObj);
	}

	obj["positions"] = positionsArray;

	QJsonDocument doc(obj);

	QFile file(calibrationPosFile);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QMessageBox::warning(this, "失败", QString("无法打开文件: %1").arg(file.errorString()));
		return;
	}

	file.write(doc.toJson(QJsonDocument::Indented));
	file.close();

	m_defaultPositionCount = m_positions.size();

	QMessageBox::information(this, "成功", "标定位置已保存！\n\n自动化流程将使用当前标定位置。");
}

void CalibrationDialog::onReadFromDevice()
{
	QPushButton* button = qobject_cast<QPushButton*>(sender());
	if (!button) return;

	int row = button->property("row").toInt();

	double x, y, z, a, b, c;
	QString errorMessage;

	if (m_pointCloudService->getDeviceMainAxisCoor(x, y, z, a, b, c, &errorMessage)) {
		QDoubleSpinBox* xSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(row, 0));
		QDoubleSpinBox* ySpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(row, 1));
		QDoubleSpinBox* zSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(row, 2));

		if (xSpinBox && ySpinBox && zSpinBox) {
			xSpinBox->setValue(x);
			ySpinBox->setValue(y);
			zSpinBox->setValue(z);

			m_positions[row].x = x;
			m_positions[row].y = y;
			m_positions[row].z = z;
		}
	} else {
		QMessageBox::warning(this, "失败", QString("获取设备坐标失败: %1").arg(errorMessage));
	}
}

void CalibrationDialog::onDeleteRow()
{
	QPushButton* button = qobject_cast<QPushButton*>(sender());
	if (!button) return;

	int row = button->property("row").toInt();

	if (m_positions.size() < 6) {
		QMessageBox::warning(this, "警告", "标定位置不能少于6组");
		return;
	}

	m_positions.removeAt(row);
	populateTable();
}

void CalibrationDialog::onOperationStarted()
{
	for (int i = 0; i < m_positions.size(); ++i)
	{
		QDoubleSpinBox* xSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 0));
		QDoubleSpinBox* ySpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 1));
		QDoubleSpinBox* zSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 2));

		if (xSpinBox && ySpinBox && zSpinBox)
		{
			m_positions[i].x = xSpinBox->value();
			m_positions[i].y = ySpinBox->value();
			m_positions[i].z = zSpinBox->value();
		}
	}

	setProgressText("开始标定...");
}

bool CalibrationDialog::performOperation()
{
	QVector<QVector3D> positions;
	for (const Position& pos : m_positions)
	{
		positions.append(QVector3D(pos.x, pos.y, pos.z));
	}

	CalibrationProgressCallback progressCallback = [this](int current, int total, const QString& status) {
		setProgressText(QString("%1 (%2/%3)").arg(status).arg(current).arg(total));
		if (total > 0) {
			setProgressValue(static_cast<int>(current * 100.0 / total));
		}
	};

	bool    ret = m_pointCloudService->executeCalibration(positions, progressCallback);
	QString errMsg;
	m_pointCloudService->machineBackHome(errMsg);
	return ret;
}

void CalibrationDialog::onOperationCompleted(bool success)
{
	if (success)
	{
		setProgressText("✅ 标定完成");
	}
	else
	{
		setProgressText("❌ 标定失败，请查看控制台获取详细信息");
	}
}

void CalibrationDialog::updateUIState()
{
	bool operationEnabled = !m_operationRunning && m_machineReady;

	if (m_addButton) {
		m_addButton->setEnabled(!m_operationRunning);
	}
	if (m_generateButton) {
		m_generateButton->setEnabled(!m_operationRunning);
	}
	if (m_resetButton) {
		m_resetButton->setEnabled(!m_operationRunning);
	}
	if (m_saveButton) {
		m_saveButton->setEnabled(!m_operationRunning);
	}
	if (m_startButton) {
		m_startButton->setEnabled(operationEnabled);
	}
	if (m_cancelButton) {
		m_cancelButton->setEnabled(!m_operationRunning);
	}

	QList<QWidget*> cellWidgets = m_tableWidget->findChildren<QWidget*>();
	for (QWidget* widget : cellWidgets) {
		QPushButton* button = qobject_cast<QPushButton*>(widget);
		if (button) {
			QString text = button->text();
			if (text == "读取" || text == "删除") {
				button->setEnabled(!m_operationRunning);
			}
		}
	}

	m_progressBar->setVisible(m_operationRunning);
}



QVector<QVector3D> CalibrationDialog::getPositions() const
{
	QVector<QVector3D> positions;
	for (const Position& pos : m_positions) {
		positions.append(QVector3D(pos.x, pos.y, pos.z));
	}
	return positions;
}
