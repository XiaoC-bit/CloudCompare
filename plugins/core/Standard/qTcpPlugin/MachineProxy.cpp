#include "MachineProxy.h"

#include "TcpClient.h"

#include <QJsonDocument>
#include <QTcpSocket>

MachineProxy::MachineProxy(QObject* parent)
    : QObject(parent)
    , m_reconnectInterval(RECONNECT_INTERVAL_MIN)
    , m_reconnectAttempts(0)
    , m_intentionalDisconnect(false)
{
	m_client = new TcpClient(this);

	m_reconnectTimer = new QTimer(this);
	m_reconnectTimer->setSingleShot(true);
	connect(m_reconnectTimer, &QTimer::timeout, this, &MachineProxy::tryConnect);

	connect(m_client, &TcpClient::connected, this, &MachineProxy::onTcpConnected);
	connect(m_client, &TcpClient::disconnected, this, &MachineProxy::onTcpDisconnected);
	connect(m_client, &TcpClient::errorOccurred, this, &MachineProxy::onTcpError);
	connect(m_client, &TcpClient::responseReceived, this, &MachineProxy::onTcpResponseReceived);

	tryConnect();
}

// ── 连接管理 ────────────────────────────────────────────────

void MachineProxy::tryConnect()
{
	if (m_client->isConnected())
		return;

	m_reconnectAttempts++;
	qDebug() << "[MachineProxy] Connecting... attempt" << m_reconnectAttempts;
	m_client->connectToHost("localhost", 20002);
}

void MachineProxy::scheduleReconnect()
{
	if (m_intentionalDisconnect)
		return;

	// 指数退避：每次失败翻倍，上限 RECONNECT_INTERVAL_MAX
	m_reconnectTimer->start(m_reconnectInterval);
	qDebug() << "[MachineProxy] Reconnecting in" << m_reconnectInterval << "ms";

	m_reconnectInterval = qMin(m_reconnectInterval * 2, RECONNECT_INTERVAL_MAX);
}

void MachineProxy::onTcpConnected()
{
	qDebug() << "[MachineProxy] Connected to machine middleware";

	// 重置退避参数
	m_reconnectInterval = RECONNECT_INTERVAL_MIN;
	m_reconnectAttempts = 0;
	m_reconnectTimer->stop();
}

void MachineProxy::onTcpDisconnected()
{
	qDebug() << "[MachineProxy] Disconnected from machine middleware";

	// 所有待处理命令立即失败，避免客户端无限等待
	failAllPendingCommands("Machine middleware disconnected");

	scheduleReconnect();
}

void MachineProxy::onTcpError(QAbstractSocket::SocketError socketError)
{
	qWarning() << "[MachineProxy] Socket error:" << socketError;

	// disconnected 信号不一定会触发（如连接从未建立），这里兜底调度重连
	if (!m_reconnectTimer->isActive())
	{
		failAllPendingCommands("Machine middleware connection error");
		scheduleReconnect();
	}
}

// ── 发送命令 ─────────────────────────────────────────────────

void MachineProxy::send(const Command& cmd)
{
	if (!m_client->isConnected())
	{
		sendErrorToSocket(cmd.socket, "Not connected to machine middleware", cmd.idCode);
		return;
	}

	// 为这条命令创建独立的超时定时器
	QTimer* cmdTimer = new QTimer(this);
	cmdTimer->setSingleShot(true);
	cmdTimer->setInterval(RESPONSE_TIMEOUT);

	QString idCode = cmd.idCode; // 捕获副本供 lambda 使用
	connect(cmdTimer, &QTimer::timeout, this, [this, idCode]()
	        { onResponseTimeout(idCode); });

	m_pendingCommands[idCode] = {cmd, cmdTimer};
	cmdTimer->start();

	m_client->send(cmd.params);
}

// ── 响应处理 ─────────────────────────────────────────────────

void MachineProxy::onTcpResponseReceived(const QJsonObject& response)
{
	QString idCode = response["IDCode"].toString();

	auto it = m_pendingCommands.find(idCode);
	if (it == m_pendingCommands.end())
		return; // 已超时或重复响应

	// 停止并销毁该命令的定时器
	it->timer->stop();
	it->timer->deleteLater();

	Command cmd = it->cmd;
	m_pendingCommands.erase(it);

	QJsonDocument doc(response);
	cmd.socket->write(doc.toJson(QJsonDocument::Compact));
	cmd.socket->flush();
}

void MachineProxy::onResponseTimeout(const QString& idCode)
{
	auto it = m_pendingCommands.find(idCode);
	if (it == m_pendingCommands.end())
		return;

	it->timer->deleteLater();
	Command cmd = it->cmd;
	m_pendingCommands.erase(it);

	sendErrorToSocket(cmd.socket, "Machine middleware response timeout", idCode);
}

// ── 工具函数 ─────────────────────────────────────────────────

void MachineProxy::failAllPendingCommands(const QString& reason)
{
	for (auto& pc : m_pendingCommands)
	{
		pc.timer->stop();
		pc.timer->deleteLater();
		sendErrorToSocket(pc.cmd.socket, reason, pc.cmd.idCode);
	}
	m_pendingCommands.clear();
}

void MachineProxy::sendErrorToSocket(QTcpSocket*    socket,
                                     const QString& message,
                                     const QString& idCode)
{
	if (socket == nullptr)
		return;
	QJsonObject err;
	err["success"] = false;
	err["message"] = message;
	if (!idCode.isEmpty())
		err["IDCode"] = idCode;

	QJsonDocument doc(err);
	socket->write(doc.toJson(QJsonDocument::Compact));
	socket->flush();
}
