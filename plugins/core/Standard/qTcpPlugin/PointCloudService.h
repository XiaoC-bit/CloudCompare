#pragma once
#include <QObject>
#include <vector>
#include <QJsonObject>
#include <QTcpSocket>

class ccMainAppInterface;

class PointCloudService : public QObject {
    Q_OBJECT
public:
    explicit PointCloudService(ccMainAppInterface* app, QObject* parent = nullptr);
    
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


	void handleFitSphere(const QJsonObject& params, QTcpSocket* socket, const QString& idCode, double& centerX, double& centerY, double& centerZ, double& rms);

  private:
    ccMainAppInterface* m_app;
    std::vector<unsigned short> m_heightBuf;
    std::vector<unsigned char> m_luminanceBuf;

    // 辅助函数
    void sendOk(QTcpSocket* socket, const QString& msg, const QString& idCode);
    void sendError(QTcpSocket* socket, const QString& msg, const QString& idCode);
    class ccHObject* getDbRoot(QTcpSocket* socket, const QString& idCode);
    class ccHObject* findByName(class ccHObject* node, const QString& name);
};
