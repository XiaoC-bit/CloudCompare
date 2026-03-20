// CcTcpServer.h
#pragma once
#include <QJsonObject>
#include <QTcpServer>
#include <QTcpSocket>

class CcTcpServer : public QTcpServer
{
	Q_OBJECT
  public:
	explicit CcTcpServer(QObject* parent = nullptr);
	bool startListening(quint16 port = 52700);

  signals:
	// 跨线程信号——Qt 自动排队到主线程
	void commandReceived(QJsonObject cmd, QTcpSocket* socket);

  public slots:
	void sendResponse(QTcpSocket* socket, bool ok, const QString& msg);

  protected:
	void incomingConnection(qintptr socketDescriptor) override;

  private slots:
	void onReadyRead();

  private:
	QByteArray m_buffer;
};
