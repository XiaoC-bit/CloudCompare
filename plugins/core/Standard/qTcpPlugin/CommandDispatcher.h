// CommandDispatcher.h
#pragma once
#include <QJsonObject>
#include <QObject>

class ccMainAppInterface;
class CcTcpServer;
class QTcpSocket;

class CommandDispatcher : public QObject
{
	Q_OBJECT
  public:
	explicit CommandDispatcher(ccMainAppInterface* app, CcTcpServer* server, QObject* parent = nullptr);

  public slots:
	void dispatch(QJsonObject cmd, QTcpSocket* socket);

  private:
	ccMainAppInterface* m_app;
	CcTcpServer* m_server;

	void handleLoad(const QJsonObject& params, QTcpSocket* socket);
	void handleFilter(const QJsonObject& params, QTcpSocket* socket);
	void handleICP(const QJsonObject& params, QTcpSocket* socket);
	void handleCamera(const QJsonObject& params, QTcpSocket* socket);
	void handleApplyViewport(const QJsonObject& params, QTcpSocket* socket);
	void handleSegment(const QJsonObject& params, QTcpSocket* socket);
	void handleDelete(const QJsonObject& params, QTcpSocket* socket);
	void handleApplyTransformation(const QJsonObject& params, QTcpSocket* socket);
	void handleClearDB(const QJsonObject& params, QTcpSocket* socket);
	

	void handleFit(const QJsonObject& params, QTcpSocket* socket);
	void handleFitSphere(const QJsonObject& params, QTcpSocket* socket);
};
