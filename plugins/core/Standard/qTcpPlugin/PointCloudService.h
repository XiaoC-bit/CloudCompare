#pragma once
#include "Eigen/Core"
#include "Eigen/Dense"

#include <QJsonObject>
#include <QMutex>
#include <QObject>
#include <QTcpSocket>
#include <QVector3D>
#include <vector>

class ccMainAppInterface;

class PointCloudService : public QObject
{
	Q_OBJECT
  public:
	friend class CalibrationDialog;

	explicit PointCloudService(ccMainAppInterface* app, QObject* parent = nullptr);
	~PointCloudService() override;

	// 命令响应函数
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
	void getStatus(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void partInspect(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);	
	//
	bool handleFitSphere(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, double& centerX, double& centerY, double& centerZ, double& rms);

  private:
	enum class MachineStatus
	{
		Idle,
		Running
	};

	struct CalibrationRigidTransform
	{
		Eigen::Matrix3d R;
		Eigen::Vector3d T;
	};

	ccMainAppInterface*         m_app;
	std::vector<unsigned short> m_heightBuf;
	std::vector<unsigned char>  m_luminanceBuf;
	QTcpSocket*                 m_machineSocket;       // 机床长连接（主线程）
	QTcpSocket*                 m_workerMachineSocket; // 机床长连接（工作线程）
	MachineStatus               m_Status;   // 状态
	QJsonObject                 m_calibrationResult;   // 标定结果
	QJsonObject                 m_inspectResult;       // 检查结果
	QString                     m_statusFilePath;      // 状态文件路径

	//标定函数
	void calibrationFunc(const QJsonObject& params);
	//工件检测函数
	void partInspectFunc(const QJsonObject& params);
	// 辅助函数
	void               sendOk(QTcpSocket* socket, const QString& msg, const QString& idCode);
	void               sendRes(QTcpSocket* socket, QJsonObject& msg, const QString& idCode);
	void               sendError(QTcpSocket* socket, const QString& msg, const QString& idCode);
	class ccHObject*   getDbRoot(QTcpSocket* socket, const QString& idCode);
    void               saveCalibrationStatus(); // 保存标定状态
    void               saveInspectStatus(); // 保存检查状态
	class ccHObject*   findByName(class ccHObject* node, const QString& name);
	void               sendResponse(QTcpSocket* socket, bool ok, const QString& msg, const QString& idCode, const QJsonObject& extra = QJsonObject());
	bool               clearDbInternal(QTcpSocket* socket, const QString& idCode);
	bool               acquirePcdInternal(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, QJsonObject* result = nullptr);
	bool               sendMachineCommand(const QJsonObject& params, QJsonObject& response, QString* errorMessage = nullptr, int timeout = 10000);
	bool               checkMachineCommandRet(const QJsonObject& response, const QString& commandName, QString* errorMessage = nullptr, const QString& messageKey = QString());
	bool               sendFileToMachine(const QString& filePath, QString* errorMessage = nullptr);
	bool               getMachineMode(QString& mode, QString* errorMessage = nullptr);
	bool               setMainProgram(QString* errorMessage = nullptr);
	bool               startMachine(QString* errorMessage = nullptr);
	bool               waitForMachineIdle(int timeoutSeconds, QString* errorMessage = nullptr);
	bool               getDeviceRun(QString& value, QString* errorMessage = nullptr);
	QVector<QVector3D> resolveCalibrationPositions(const QJsonObject& params, QString* errorMessage = nullptr) const;
	void               saveCalibrationStatus();

	bool ensureConnected(QString* errorMessage, int connectTimeout);
	int  findJsonObjectEnd(const QByteArray& buffer);
	void resetConnection();
	void setError(QString* out, const QString& msg);

	static CalibrationRigidTransform computeRigidTransform(const std::vector<Eigen::Vector3d>& scanner_points,
	                                                       const std::vector<Eigen::Vector3d>& machine_points);
	static Eigen::Matrix4d           toMatrix4d(const CalibrationRigidTransform& tf);
};
