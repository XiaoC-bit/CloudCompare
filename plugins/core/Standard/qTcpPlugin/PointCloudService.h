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
	void getPartInspectResult(const QJsonObject& params, QTcpSocket* socket, const QString& idCode); // 获取工件检查结果
	
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
	Eigen::Matrix4d             m_calibrationMatrix;   // 标定结果矩阵
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
    void               savePartInspectResult(const QString& rfid, const QJsonObject& result); // 保存工件检查结果
    // 内部函数
    bool               loadInternal(const QJsonObject& params, QString* errorMessage); // 内部加载函数
    bool               applyViewportInternal(const QJsonObject& params, QString* errorMessage); // 内部视口应用函数
    bool               segmentPolygonInternal(const QJsonObject& params, QString* errorMessage); // 内部分割函数（基于多边形）
    bool               deleteObjectInternal(const QJsonObject& params, QString* errorMessage); // 内部删除对象函数
    bool               mergeInternal(const QJsonObject& params, QString* errorMessage); // 内部合并函数
    bool               icpInternal(const QJsonObject& params, QString* errorMessage); // 内部ICP配准函数
    
    // 数学工具函数
    static constexpr double PI = 3.14159265358979323846;
    static inline double deg2rad(double deg) { return deg * PI / 180.0; }
    static inline double rad2deg(double rad) { return rad * 180.0 / PI; }
    static Eigen::Matrix4d invertRigid(const Eigen::Matrix4d& T);
    static Eigen::Matrix4d makePivotTransform(const Eigen::Matrix3d& Rot, const Eigen::Vector3d& pivot);
    static Eigen::Matrix4d computeCameraMotion(const Eigen::Matrix4d& T_cam2robot, double x, double y, double z, double B_deg, double C_deg);
    static Eigen::Matrix4d buildRobotMotion(double x, double y, double z, double B_deg, double C_deg, const Eigen::Vector3d& pivot_B, const Eigen::Vector3d& pivot_C);
    // 接口函数
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


	bool ensureConnected(QString* errorMessage, int connectTimeout);
	int  findJsonObjectEnd(const QByteArray& buffer);
	void resetConnection();
	void setError(QString* out, const QString& msg);

	static CalibrationRigidTransform computeRigidTransform(const std::vector<Eigen::Vector3d>& scanner_points,
	                                                       const std::vector<Eigen::Vector3d>& machine_points);
	static Eigen::Matrix4d           toMatrix4d(const CalibrationRigidTransform& tf);
};
