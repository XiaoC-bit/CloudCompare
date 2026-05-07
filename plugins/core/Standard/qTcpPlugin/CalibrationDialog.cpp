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
	setFixedSize(450, 400);
}

CalibrationDialog::~CalibrationDialog()
{
}

void CalibrationDialog::setupAdditionalUI()
{
	m_tableWidget = new QTableWidget(this);
	m_tableWidget->setColumnCount(4);
	m_tableWidget->setHorizontalHeaderLabels({"X", "Y", "Z", "操作"});
	m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_tableWidget->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
	m_tableWidget->setColumnWidth(3, 80);
	m_mainLayout->addWidget(m_tableWidget);

	m_addButton = new QPushButton("新增位置", this);
	connect(m_addButton, &QPushButton::clicked, this, &CalibrationDialog::onAddPosition);
	m_mainLayout->addWidget(m_addButton);

	m_buttonLayout = new QHBoxLayout();
	m_resetButton = new QPushButton("复位", this);
	connect(m_resetButton, &QPushButton::clicked, this, &CalibrationDialog::onReset);
	m_startButton = new QPushButton("开始标定", this);
	connect(m_startButton, &QPushButton::clicked, this, &MachineStatusDialog::onStartOperation);
	m_cancelButton = new QPushButton("取消", this);
	connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);

	m_buttonLayout->addWidget(m_resetButton);
	m_buttonLayout->addWidget(m_startButton);
	m_buttonLayout->addWidget(m_cancelButton);
	m_mainLayout->addLayout(m_buttonLayout);

	populateTable();
}

void CalibrationDialog::populateTable()
{
	m_tableWidget->setRowCount(m_positions.size());

	for (int i = 0; i < m_positions.size(); ++i) {
		const Position& pos = m_positions[i];

		QDoubleSpinBox* xSpinBox = new QDoubleSpinBox();
		xSpinBox->setMinimum(-1000);
		xSpinBox->setMaximum(1000);
		xSpinBox->setDecimals(3);
		xSpinBox->setValue(pos.x);
		m_tableWidget->setCellWidget(i, 0, xSpinBox);

		QDoubleSpinBox* ySpinBox = new QDoubleSpinBox();
		ySpinBox->setMinimum(-1000);
		ySpinBox->setMaximum(1000);
		ySpinBox->setDecimals(3);
		ySpinBox->setValue(pos.y);
		m_tableWidget->setCellWidget(i, 1, ySpinBox);

		QDoubleSpinBox* zSpinBox = new QDoubleSpinBox();
		zSpinBox->setMinimum(-1000);
		zSpinBox->setMaximum(1000);
		zSpinBox->setDecimals(3);
		zSpinBox->setValue(pos.z);
		m_tableWidget->setCellWidget(i, 2, zSpinBox);

		QPushButton* deleteButton = new QPushButton("删除");
		deleteButton->setProperty("row", i);
		connect(deleteButton, &QPushButton::clicked, this, &CalibrationDialog::onDeleteRow);
		m_tableWidget->setCellWidget(i, 3, deleteButton);
	}
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
