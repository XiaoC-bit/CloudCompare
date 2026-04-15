#pragma once

#include <QDialog>
#include <QVector3D>
#include <QTcpSocket>
#include <QProgressBar>
#include <QLabel>
#include "Eigen/Dense"
#include "Eigen/Core"

class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QTableWidget;
class PointCloudService;
class ccMainAppInterface;

class CalibrationDialog : public QDialog
{
private:
	friend class CalibrationGuard;
    struct Position
    {
        double x;
        double y;
        double z;
        
        Position(double x_, double y_, double z_)
		    : x(x_)
		    , y(y_)
		    , z(z_)
		{
		}
    };
    
    struct RigidTransform
    {
        Eigen::Matrix3d R; // 旋转矩阵
        Eigen::Vector3d T; // 平移向量
    };

public:
	explicit CalibrationDialog(ccMainAppInterface*app,PointCloudService* pointCloudService, QWidget* parent = nullptr);
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
    bool connectToMachine();
    bool sendCommand(const QJsonObject &params, QJsonObject &response, int timeout = 5000);
    bool sendFileToMachine(const QString &filePath);
    bool startMachine();
    bool waitForMachineIdle(const int &timeOut = 120);
    bool acquirePointCloud(const QString &outputName);
    bool setMainProgram();
    bool getMode(QString &mode);
    
    // 将 RigidTransform 转换为 4×4 齐次变换矩阵
    static Eigen::Matrix4d toMatrix4d(const RigidTransform& tf);
    
    // 输入：
    //   scanner_points  扫描仪坐标系下的 N 个球心
    //   machine_points  机床坐标系下对应的 N 个球心
    // 输出：
    //   RigidTransform {R, T}，满足 machine = R * scanner + T
    static RigidTransform computeRigidTransform(
        const std::vector<Eigen::Vector3d>& scanner_points,
        const std::vector<Eigen::Vector3d>& machine_points);
    
    QVBoxLayout *m_mainLayout;
    QTableWidget *m_tableWidget;
    QPushButton *m_addButton;
    QPushButton *m_resetButton;
    QPushButton *m_startButton;
    QPushButton *m_cancelButton;
    QHBoxLayout *m_buttonLayout;
    QProgressBar *m_progressBar;
    QLabel *m_progressLabel;
    
    QTcpSocket *m_socket;
    PointCloudService *m_pointCloudService;
    QVector<Position> m_positions;
    static const QVector<Position> DEFAULT_POSITIONS;
    static const int MAX_POSITIONS = 30;
    static const int MACHINE_PORT = 20002;
    static const QString MACHINE_HOST;

	ccMainAppInterface* m_app;

	
    std::vector<unsigned short> m_heightBuf;
	std::vector<unsigned char>  m_luminanceBuf;

	void setCalibrationRunning(bool running);
	bool m_calibrationRunning = false;

  protected:
	void closeEvent(QCloseEvent* event) override;
};




// 在 .h 中或匿名命名空间里
class CalibrationGuard
{
  public:
	CalibrationDialog* dlg;
	explicit CalibrationGuard(CalibrationDialog* d)
	    : dlg(d)
	{
		dlg->setCalibrationRunning(true);
	}
	~CalibrationGuard()
	{
		dlg->setCalibrationRunning(false);
	}
};
