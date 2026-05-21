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

const QVector<CalibrationDialog::Position> CalibrationDialog::DEFAULT_POSITIONS = {
    {0, 0, 0},
    {5, 0, 0},
    {5, 5, 0},
    {5, 10, 0},
    {10, 10, 0},
    {0, 0, -2.5},
    {0, 0, -5}
};

CalibrationDialog::CalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : MachineStatusDialog(app, pointCloudService, parent)
    , m_positions(DEFAULT_POSITIONS)
{
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

	// --- 底部按钮 ---
	m_buttonLayout = new QHBoxLayout();
	m_buttonLayout->setSpacing(8);

	m_resetButton = new QPushButton("复位", this);
	m_resetButton->setFixedWidth(80);
	m_resetButton->setFixedHeight(32);
	connect(m_resetButton, &QPushButton::clicked, this, &CalibrationDialog::onReset);
	m_buttonLayout->addWidget(m_resetButton);

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

		auto makeSpinBox = [](double value) -> QDoubleSpinBox*
		{
			QDoubleSpinBox* sb = new QDoubleSpinBox();
			sb->setMinimum(-1000);
			sb->setMaximum(1000);
			sb->setDecimals(3);
			sb->setValue(value);
			sb->setButtonSymbols(QAbstractSpinBox::NoButtons); // 去掉上下箭头，更整洁
			sb->setFrame(false);
			sb->setAlignment(Qt::AlignCenter);
			sb->setStyleSheet("background: transparent; padding: 0 4px;");
			return sb;
		};

		m_tableWidget->setCellWidget(i, 0, makeSpinBox(pos.x));
		m_tableWidget->setCellWidget(i, 1, makeSpinBox(pos.y));
		m_tableWidget->setCellWidget(i, 2, makeSpinBox(pos.z));

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
	if (m_positions.size() >= MAX_POSITIONS) {
		QMessageBox::warning(this, "警告", "最多只能添加30组数据");
		return;
	}

	m_positions.append(Position(0, 0, 0));
	populateTable();
}

void CalibrationDialog::onReset()
{
	m_positions = DEFAULT_POSITIONS;
	populateTable();
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

	if (m_positions.size() <= DEFAULT_POSITIONS.size()) {
		QMessageBox::warning(this, "警告", "数据组数不能少于默认值数量");
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

	return m_pointCloudService->executeCalibration(positions);
}

void CalibrationDialog::onOperationCompleted(bool success)
{
	if (success)
	{
		setProgressText("✅ 标定完成");
		accept();
	}
	else
	{
		setProgressText("❌ 标定失败，请查看控制台获取详细信息");
	}
}

QVector<QVector3D> CalibrationDialog::getDefaultPositions()
{
	QVector<QVector3D> positions;
	for (const Position& pos : DEFAULT_POSITIONS) {
		positions.append(QVector3D(pos.x, pos.y, pos.z));
	}
	return positions;
}

QVector<QVector3D> CalibrationDialog::getPositions() const
{
	QVector<QVector3D> positions;
	for (const Position& pos : m_positions) {
		positions.append(QVector3D(pos.x, pos.y, pos.z));
	}
	return positions;
}
