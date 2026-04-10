#include "CalibrationDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QMessageBox>

const QVector<CalibrationDialog::Position> CalibrationDialog::DEFAULT_POSITIONS = {
    {0, 0, 0},
    {5, 0, 0},
    {5, 5, 0},
    {5, 10, 0},
    {10, 10, 0},
    {0, 0, 2.5},
    {0, 0, 5}
};

CalibrationDialog::CalibrationDialog(QWidget *parent) 
    : QDialog(parent)
    , m_positions(DEFAULT_POSITIONS)
{
    setWindowTitle("标定");
    setFixedSize(400, 400);
    setupUI();
    populateTable();
}

CalibrationDialog::~CalibrationDialog()
{
}

void CalibrationDialog::setupUI()
{
    m_mainLayout = new QVBoxLayout(this);
    
    m_tableWidget = new QTableWidget(this);
    m_tableWidget->setColumnCount(3);
    m_tableWidget->setHorizontalHeaderLabels({"X", "Y", "Z"});
    m_tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_mainLayout->addWidget(m_tableWidget);
    
    m_addButton = new QPushButton("新增位置", this);
    connect(m_addButton, &QPushButton::clicked, this, &CalibrationDialog::onAddPosition);
    m_mainLayout->addWidget(m_addButton);
    
    m_buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton("开始标定", this);
    connect(m_startButton, &QPushButton::clicked, this, &CalibrationDialog::onStartCalibration);
    m_cancelButton = new QPushButton("取消", this);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    m_buttonLayout->addWidget(m_startButton);
    m_buttonLayout->addWidget(m_cancelButton);
    m_mainLayout->addLayout(m_buttonLayout);
}

void CalibrationDialog::populateTable()
{
    m_tableWidget->setRowCount(m_positions.size());
    
    for (int i = 0; i < m_positions.size(); ++i) {
        const Position &pos = m_positions[i];
        
        QDoubleSpinBox *xSpinBox = new QDoubleSpinBox();
        xSpinBox->setValue(pos.x);
        xSpinBox->setMinimum(-1000);
        xSpinBox->setMaximum(1000);
        xSpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 0, xSpinBox);
        
        QDoubleSpinBox *ySpinBox = new QDoubleSpinBox();
        ySpinBox->setValue(pos.y);
        ySpinBox->setMinimum(-1000);
        ySpinBox->setMaximum(1000);
        ySpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 1, ySpinBox);
        
        QDoubleSpinBox *zSpinBox = new QDoubleSpinBox();
        zSpinBox->setValue(pos.z);
        zSpinBox->setMinimum(-1000);
        zSpinBox->setMaximum(1000);
        zSpinBox->setDecimals(3);
        m_tableWidget->setCellWidget(i, 2, zSpinBox);
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

void CalibrationDialog::onStartCalibration()
{
    // 保存当前表格中的值到m_positions
    for (int i = 0; i < m_positions.size(); ++i) {
        QDoubleSpinBox *xSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 0));
        QDoubleSpinBox *ySpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 1));
        QDoubleSpinBox *zSpinBox = static_cast<QDoubleSpinBox*>(m_tableWidget->cellWidget(i, 2));
        
        if (xSpinBox && ySpinBox && zSpinBox) {
            m_positions[i].x = xSpinBox->value();
            m_positions[i].y = ySpinBox->value();
            m_positions[i].z = zSpinBox->value();
        }
    }
    
    accept();
}

QVector<QVector3D> CalibrationDialog::getDefaultPositions()
{
    QVector<QVector3D> positions;
    for (const Position &pos : DEFAULT_POSITIONS) {
        positions.append(QVector3D(pos.x, pos.y, pos.z));
    }
    return positions;
}

QVector<QVector3D> CalibrationDialog::getPositions() const
{
    QVector<QVector3D> positions;
    for (const Position &pos : m_positions) {
        positions.append(QVector3D(pos.x, pos.y, pos.z));
    }
    return positions;
}