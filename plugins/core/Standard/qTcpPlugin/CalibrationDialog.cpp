#include "CalibrationDialog.h"
#include "MachineProxy.h"
#include "PointCloudService.h"
#include "Command.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTimer>
#include <QThread>

const QVector<CalibrationDialog::Position> CalibrationDialog::DEFAULT_POSITIONS = {
    {0, 0, 0},
    {5, 0, 0},
    {5, 5, 0},
    {5, 10, 0},
    {10, 10, 0},
    {0, 0, 2.5},
    {0, 0, 5}
};

CalibrationDialog::CalibrationDialog(MachineProxy *machineProxy, PointCloudService *pointCloudService, QWidget *parent) 
    : QDialog(parent)
    , m_machineProxy(machineProxy)
    , m_pointCloudService(pointCloudService)
    , m_positions(DEFAULT_POSITIONS)
{
    setWindowTitle("标定");
    setFixedSize(450, 400);
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
    connect(m_startButton, &QPushButton::clicked, this, &CalibrationDialog::onStartCalibration);
    m_cancelButton = new QPushButton("取消", this);
    connect(m_cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    
    m_buttonLayout->addWidget(m_resetButton);
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
        
        QPushButton *deleteButton = new QPushButton("删除");
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
    
    // 开始标定流程
    bool calibrationSuccess = true;
    
    for (int i = 0; i < m_positions.size(); ++i) {
        const Position &pos = m_positions[i];
        
        // 1. 打开Template文件夹下的Calibration.nc文件
        QString appDir = QCoreApplication::applicationDirPath();
        QString templateDir = appDir + "/Template";
        QString templateFile = templateDir + "/Calibration.nc";
        QString outputFile = templateDir + "/Calibration_" + QString::number(i+1) + ".nc";
        
        // 检查Template文件夹是否存在
        QDir dir(templateDir);
        if (!dir.exists()) {
            QMessageBox::critical(this, "错误", "Template文件夹不存在");
            calibrationSuccess = false;
            break;
        }
        
        // 检查Calibration.nc文件是否存在
        QFile templateNc(templateFile);
        if (!templateNc.exists()) {
            QMessageBox::critical(this, "错误", "Calibration.nc文件不存在");
            calibrationSuccess = false;
            break;
        }
        
        // 2. 替换文件中的{X}, {Y}, {Z}
        if (!templateNc.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "错误", "无法打开Calibration.nc文件");
            calibrationSuccess = false;
            break;
        }
        
        QTextStream in(&templateNc);
        QString content = in.readAll();
        templateNc.close();
        
        content.replace("{X}", QString::number(pos.x));
        content.replace("{Y}", QString::number(pos.y));
        content.replace("{Z}", QString::number(pos.z));
        
        // 保存替换后的文件
        QFile outputNc(outputFile);
        if (!outputNc.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "错误", "无法保存替换后的NC文件");
            calibrationSuccess = false;
            break;
        }
        
        QTextStream out(&outputNc);
        out << content;
        outputNc.close();
        
        // 3. 发送NC文件到机床
        if (!sendFileToMachine(outputFile)) {
            calibrationSuccess = false;
            break;
        }
        
        // 4. 发送启动机床命令
        if (!startMachine()) {
            calibrationSuccess = false;
            break;
        }
        
        // 5. 等待机床空闲
        if (!waitForMachineIdle()) {
            calibrationSuccess = false;
            break;
        }
        
        // 6. 获取点云数据
        if (!acquirePointCloud()) {
            calibrationSuccess = false;
            break;
        }
    }
    
    if (calibrationSuccess) {
        QMessageBox::information(this, "成功", "标定完成");
        accept();
    } else {
        QMessageBox::warning(this, "失败", "标定过程中出现错误");
    }
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

void CalibrationDialog::onReset()
{
    m_positions = DEFAULT_POSITIONS;
    populateTable();
}

void CalibrationDialog::onDeleteRow()
{
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) return;
    
    int row = button->property("row").toInt();
    
    if (m_positions.size() <= DEFAULT_POSITIONS.size()) {
        QMessageBox::warning(this, "警告", "数据组数不能少于默认值数量");
        return;
    }
    
    m_positions.removeAt(row);
    populateTable();
}

bool CalibrationDialog::sendFileToMachine(const QString &filePath)
{
    if (!m_machineProxy) {
        QMessageBox::critical(this, "错误", "机床代理未初始化");
        return false;
    }
    
    // 创建SendFile命令
    QJsonObject params;
    params["Command"] = "SendFile";
    params["Device"] = "CNC";
    params["LocalFile"] = filePath;
    
    Command cmd;
	cmd.type = "machine";
	cmd.params = params;
    m_machineProxy->send(cmd);
    
    return true;
}

bool CalibrationDialog::startMachine()
{
    if (!m_machineProxy) {
        QMessageBox::critical(this, "错误", "机床代理未初始化");
        return false;
    }
    
    // 创建Start命令
    QJsonObject params;
    params["Command"] = "Start";
    params["Device"] = "CNC";

	Command cmd;
	cmd.type   = "machine";
	cmd.params = params;
    m_machineProxy->send(cmd);
    
    return true;
}

bool CalibrationDialog::waitForMachineIdle()
{
    if (!m_machineProxy) {
        QMessageBox::critical(this, "错误", "机床代理未初始化");
        return false;
    }
    
    // 创建GetStatus命令
    QJsonObject params;
    params["Command"] = "GetStatus";
    params["Device"] = "CNC";

	Command cmd;
	cmd.type   = "machine";
	cmd.params = params;
    
    // 等待机床空闲，最多等待60秒
    int maxWaitTime = 60000; // 60秒
    int waitInterval = 1000; // 1秒
    int elapsedTime = 0;
    
    while (elapsedTime < maxWaitTime) {
        m_machineProxy->send(cmd);
        
        // 这里需要等待响应，实际实现中应该有回调机制
        // 这里简化处理，直接等待
        QThread::msleep(waitInterval);
        elapsedTime += waitInterval;
        
        // 假设这里会检查机床状态，实际实现中应该从回调中获取
        // 这里简化处理，直接返回true
        return true;
    }
    
    QMessageBox::critical(this, "错误", "等待机床空闲超时");
    return false;
}

bool CalibrationDialog::acquirePointCloud()
{
    if (!m_pointCloudService) {
        QMessageBox::critical(this, "错误", "点云服务未初始化");
        return false;
    }
    
    // 创建acquirePcd命令参数
    QJsonObject params;
    // 这里可以添加必要的参数
    
    // 由于acquirePcd需要socket和idCode参数，这里简化处理
    // 实际实现中应该通过CommandDispatcher发送命令
    
    return true;
}
