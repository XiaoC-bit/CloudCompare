// CommandDispatcher.h
#pragma once
#include <QJsonObject>
#include <QObject>

class ccMainAppInterface;
class CcTcpServer;
class QTcpSocket;
class ccHObject;

class CommandDispatcher : public QObject
{
	Q_OBJECT
public:
	explicit CommandDispatcher(ccMainAppInterface* app, CcTcpServer* server, QObject* parent = nullptr);

public slots:
	void dispatch(QJsonObject cmd, QTcpSocket* socket);

private:
	ccMainAppInterface* m_app;
	CcTcpServer*        m_server;

	// Persistent acquisition buffers — reallocated only when size grows
	std::vector<unsigned short> m_heightBuf;
	std::vector<unsigned char>  m_luminanceBuf;

	// Helpers
	void       sendOk(QTcpSocket* socket, const QString& msg, const QString& idCode);
	void       sendError(QTcpSocket* socket, const QString& msg, const QString& idCode);
	ccHObject* getDbRoot(QTcpSocket* socket, const QString& idCode);
	ccHObject* findByName(ccHObject* node, const QString& name);

	// Handlers
	void handleLoad(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleFilter(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleICP(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleCamera(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleApplyViewport(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleSegment(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleDelete(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleApplyTransformation(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleClearDB(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleFit(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleFitSphere(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleSubsample(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleMerge(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleClone(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
	void handleAcquirePcd(const QJsonObject& params, QTcpSocket* socket, const QString& idCode);
};
