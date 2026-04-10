#pragma once
#include "Command.h"

#include <QAbstractSocket>
#include <QMap>
#include <QObject>
#include <QTimer>
#include <qmutex.h>
#include <qwaitcondition.h>

class TcpClient;

class MachineProxy : public QObject
{
	Q_OBJECT
  public:
	explicit MachineProxy(QObject* parent = nullptr);
	void send(const Command& cmd);
	bool sendSync(const QJsonObject& params, QJsonObject& response, int timeout = 5000);

  private slots:
	void onTcpConnected();
	void onTcpDisconnected();
	void onTcpError(QAbstractSocket::SocketError socketError);
	void onTcpResponseReceived(const QJsonObject& response);
	void onResponseTimeout(const QString& idCode);
	void tryConnect();

  private:
	struct PendingCommand
	{
		Command cmd;
		QTimer* timer; // 每个命令独立的超时定时器
	};

	struct SyncPendingCommand
	{
		QJsonObject response;
		bool received = false;
		QMutex mutex;
		QWaitCondition cond;
	};

	TcpClient*                    m_client;
	QMap<QString, PendingCommand> m_pendingCommands;
	QMutex m_syncMapMutex;
	QMap<QString, SyncPendingCommand*> m_syncPendingCommands;

	// 重连相关
	QTimer* m_reconnectTimer;
	int     m_reconnectInterval;     // 当前重连间隔（ms），用于退避
	int     m_reconnectAttempts;     // 当前重连次数
	bool    m_intentionalDisconnect; // 是否主动断开（预留）

	static const int RESPONSE_TIMEOUT       = 5000;  // 命令响应超时（ms）
	static const int RECONNECT_INTERVAL_MIN = 2000;  // 最小重连间隔（ms）
	static const int RECONNECT_INTERVAL_MAX = 30000; // 最大重连间隔（ms）
	static const int RECONNECT_MAX_ATTEMPTS = 0;     // 0 = 无限重试

	void scheduleReconnect();
	void failAllPendingCommands(const QString& reason);
	void sendErrorToSocket(QTcpSocket* socket, const QString& message, const QString& idCode = "");
};
