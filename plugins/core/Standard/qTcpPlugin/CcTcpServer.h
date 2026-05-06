#pragma once

#include <QMap>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

class CommandParser;
class CommandDispatcher;

struct SocketBuffer
{
	QByteArray data;
	QTimer*    timeoutTimer = nullptr;
};

class CcTcpServer : public QTcpServer
{
	Q_OBJECT

  public:
	explicit CcTcpServer(QObject* parent = nullptr);
	~CcTcpServer();

	bool startListening(quint16 port = 52700);
	void setCommandDispatcher(CommandDispatcher* dispatcher);

  protected:
	void incomingConnection(qintptr socketDescriptor) override;

  private slots:
	void onReadyRead();
	void onDisconnected();

  private:
	void processBuffer(QTcpSocket* socket);
	void cleanupSocket(QTcpSocket* socket);

	QMap<QTcpSocket*, SocketBuffer> m_buffers;
	CommandParser*                  m_parser;
	CommandDispatcher*              m_dispatcher;

	static constexpr int MAX_BUFFER_SIZE   = 4 * 1024 * 1024; // 4MB
	static constexpr int BUFFER_TIMEOUT_MS = 5000;            // 5秒
};
