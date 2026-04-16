#pragma once
#include <QObject>
#include <vector>
#include <QJsonObject>
#include <QVector3D>
#include <QTcpSocket>
#include <QMutex>

#include "Eigen/Core"
#include "Eigen/Dense"

class ccMainAppInterface;

class PointCloudService : public QObject {
    Q_OBJECT
public:

	friend class CalibrationDialog;

    explicit PointCloudService(ccMainAppInterface* app, QObject* parent = nullptr);
    ~PointCloudService() override;
    
    // 点云处理函数
    void load(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void filter(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void icp(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void camera(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void applyViewport(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void applyTransformation(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void segment(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void deleteObject(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void fit(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void clearDB(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void subsample(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void merge(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void clone(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void acquirePcd(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
    void startCalibration(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void calibrationFunc(const QJsonObject& params);
    void getStatus(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);


	bool handleFitSphere(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, double& centerX, double& centerY, double& centerZ, double& rms);

  private:

    enum class CalibrationStatus {
        Idle,
		Running
    };

    struct CalibrationRigidTransform
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
    };

    ccMainAppInterface* m_app;
    std::vector<unsigned short> m_heightBuf;
    std::vector<unsigned char> m_luminanceBuf;
    QTcpSocket* m_machineSocket; // 机床长连接（主线程）
    QTcpSocket* m_workerMachineSocket; // 机床长连接（工作线程）
    CalibrationStatus m_calibrationStatus; // 标定状态
    QJsonObject m_calibrationResult; // 标定结果
    QString m_statusFilePath; // 状态文件路径

    // 辅助函数
	void                             sendOk(QTcpSocket* socket, const QString& msg, const QString& idCode);
	void                             sendRes(QTcpSocket* socket, QJsonObject& msg, const QString& idCode);
    void sendError(QTcpSocket* socket, const QString& msg, const QString& idCode);
    class ccHObject* getDbRoot(QTcpSocket* socket, const QString& idCode);
    class ccHObject* findByName(class ccHObject* node, const QString& name);
    void sendResponse(QTcpSocket* socket, bool ok, const QString& msg, const QString& idCode, const QJsonObject& extra = QJsonObject());
    bool clearDbInternal(QTcpSocket* socket, const QString& idCode);
    bool acquirePcdInternal(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, QJsonObject* result = nullptr);
    bool sendMachineCommand(const QJsonObject& params, QJsonObject& response, QString* errorMessage = nullptr, int timeout = 10000);
	bool                             checkMachineCommandRet(const QJsonObject& response, const QString& commandName, QString* errorMessage = nullptr, const QString& messageKey = QString());
    bool sendFileToMachine(const QString& filePath, QString* errorMessage = nullptr);
    bool getMachineMode(QString& mode, QString* errorMessage = nullptr);
    bool setMainProgram(QString* errorMessage = nullptr);
    bool startMachine(QString* errorMessage = nullptr);
    bool waitForMachineIdle(int timeoutSeconds, QString* errorMessage = nullptr);
	bool                             getDeviceRun(QString& value, QString* errorMessage = nullptr);
    QVector<QVector3D> resolveCalibrationPositions(const QJsonObject& params, QString* errorMessage = nullptr) const;
    static CalibrationRigidTransform computeRigidTransform(const std::vector<Eigen::Vector3d>& scanner_points,
                                                           const std::vector<Eigen::Vector3d>& machine_points);
    static Eigen::Matrix4d toMatrix4d(const CalibrationRigidTransform& tf);
    void saveCalibrationStatus();

	bool PointCloudService::ensureConnected(QString* errorMessage, int connectTimeout);

	int PointCloudService::findJsonObjectEnd(const QByteArray& buffer);

	void PointCloudService::resetConnection();

	void PointCloudService::setError(QString* out, const QString& msg);

};
