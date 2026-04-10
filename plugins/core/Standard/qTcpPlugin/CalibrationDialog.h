#pragma once

#include <QDialog>
#include <QVector3D>

class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QTableWidget;
class MachineProxy;
class PointCloudService;

class CalibrationDialog : public QDialog
{
private:
    struct Position
    {
        double x;
        double y;
        double z;
        
        Position(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    };

public:
    explicit CalibrationDialog(MachineProxy *machineProxy, PointCloudService *pointCloudService, QWidget *parent = nullptr);
    ~CalibrationDialog() override;
    
    static QVector<QVector3D> getDefaultPositions();
    QVector<QVector3D> getPositions() const;

private slots:
    void onAddPosition();
    void onStartCalibration();
    void onReset();
    void onDeleteRow();

private:
    void setupUI();
    void populateTable();
    bool sendFileToMachine(const QString &filePath);
    bool startMachine();
    bool waitForMachineIdle();
    bool acquirePointCloud();
    
    QVBoxLayout *m_mainLayout;
    QTableWidget *m_tableWidget;
    QPushButton *m_addButton;
    QPushButton *m_resetButton;
    QPushButton *m_startButton;
    QPushButton *m_cancelButton;
    QHBoxLayout *m_buttonLayout;
    
    MachineProxy *m_machineProxy;
    PointCloudService *m_pointCloudService;
    QVector<Position> m_positions;
    static const QVector<Position> DEFAULT_POSITIONS;
    static const int MAX_POSITIONS = 30;
};