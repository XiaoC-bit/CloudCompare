#include "CcTcpServer.h"

#include "AppLogger.h"
#include "CommLogger.h"
#include "CommandDispatcher.h"
#include "CommandParser.h"

// ─────────────────────────────────────────────
//  构造 / 析构
// ─────────────────────────────────────────────

CcTcpServer::CcTcpServer(QObject* parent)
    : QTcpServer(parent)
    , m_parser(new CommandParser())
    , m_dispatcher(nullptr)
{
}

CcTcpServer::~CcTcpServer()
{
	// 清理所有残留 socket buffer
	for (auto& sb : m_buffers)
	{
		if (sb.timeoutTimer)
		{
			sb.timeoutTimer->stop();
			sb.timeoutTimer->deleteLater();
		}
	}
	m_buffers.clear();
	delete m_parser;
}

// ─────────────────────────────────────────────
//  公有接口
// ─────────────────────────────────────────────

bool CcTcpServer::startListening(quint16 port)
{
	return listen(QHostAddress::Any, port);
}

void CcTcpServer::setCommandDispatcher(CommandDispatcher* dispatcher)
{
	m_dispatcher = dispatcher;
}

// ─────────────────────────────────────────────
//  新连接
// ─────────────────────────────────────────────

void CcTcpServer::incomingConnection(qintptr socketDescriptor)
{
	QTcpSocket* socket = new QTcpSocket(this);
	if (!socket->setSocketDescriptor(socketDescriptor))
	{
		QString err = QString("Failed to set socket descriptor: %1").arg(socket->errorString());
		LOG_RECEIVED(err);
		UI_LOG_NET_ERR(err);
		delete socket;
		return;
	}

	QString clientAddr = QString("%1:%2").arg(socket->peerAddress().toString()).arg(socket->peerPort());
	UI_LOG_NET(QString("客户端连接: %1").arg(clientAddr));

	// 为该 socket 创建独立的超时 timer
	QTimer* timer = new QTimer(this);
	timer->setSingleShot(true);
	timer->setInterval(BUFFER_TIMEOUT_MS);

	connect(timer, &QTimer::timeout, this, [this, socket]()
	        {
        if (!m_buffers.contains(socket)) return;
        QByteArray& buf = m_buffers[socket].data;
        if (!buf.isEmpty()) {
			QString msg = QString("Buffer timeout, discarding %1 bytes of incomplete JSON").arg(buf.size());
			LOG_RECEIVED(msg);
			UI_LOG_NET_WARN(msg);
            buf.clear();
        } });

	SocketBuffer sb;
	sb.timeoutTimer = timer;
	m_buffers.insert(socket, sb);

	connect(socket, &QTcpSocket::readyRead, this, &CcTcpServer::onReadyRead);
	connect(socket, &QTcpSocket::disconnected, this, &CcTcpServer::onDisconnected);
	connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
}

// ─────────────────────────────────────────────
//  数据到达
// ─────────────────────────────────────────────

void CcTcpServer::onReadyRead()
{
	QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
	if (!socket || !m_buffers.contains(socket))
		return;

	SocketBuffer& sb = m_buffers[socket];
	sb.data.append(socket->readAll());

	// 每次收到数据都重置超时计时
	sb.timeoutTimer->start();

	// 防止恶意或异常客户端撑爆内存
	if (sb.data.size() > MAX_BUFFER_SIZE)
	{
		const QString msg = QString("缓冲区超限 %1 MB，断开连接").arg(MAX_BUFFER_SIZE / 1024 / 1024);
		LOG_RECEIVED(msg);
		UI_LOG_NET_ERR(msg);
		cleanupSocket(socket);
		socket->disconnectFromHost();
		return;
	}

	processBuffer(socket);
}

// ─────────────────────────────────────────────
//  断开连接
// ─────────────────────────────────────────────

void CcTcpServer::onDisconnected()
{
	QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
	if (!socket)
		return;
	const QString addr = QString("%1:%2")
	                         .arg(socket->peerAddress().toString())
	                         .arg(socket->peerPort());
	UI_LOG_NET(QString("客户端断开: %1").arg(addr));
	cleanupSocket(socket);
}

// ─────────────────────────────────────────────
//  私有：解析 buffer 中的完整 JSON
// ─────────────────────────────────────────────

void CcTcpServer::processBuffer(QTcpSocket* socket)
{
	QByteArray& buf = m_buffers[socket].data;

	int depth     = 0;
	int jsonStart = -1;

	for (int i = 0; i < buf.size(); ++i)
	{
		const char c = buf[i];

		if (c == '{')
		{
			if (depth == 0)
				jsonStart = i;
			++depth;
		}
		else if (c == '}')
		{
			if (depth == 0)
			{
				// 多余的 '}'，说明之前数据已损坏，丢弃到此处
				LOG_RECEIVED("Unexpected '}', discarding corrupted data");
				buf.remove(0, i + 1);
				i         = -1;
				jsonStart = -1;
				continue;
			}

			--depth;

			if (depth == 0 && jsonStart != -1)
			{
				// 提取一条完整 JSON
				const QByteArray jsonBytes = buf.mid(jsonStart, i - jsonStart + 1);
				const QString    jsonStr   = QString::fromUtf8(jsonBytes);
				LOG_RECEIVED(jsonStr);

				// 取 type 字段作为摘要显示在 UI 日志
				const QString addr = QString("%1:%2")
				                         .arg(socket->peerAddress().toString())
				                         .arg(socket->peerPort());

				try
				{
					Command cmd = m_parser->parse(jsonStr);
					cmd.socket  = socket;

					const QString cmdType = QString::fromStdString(cmd.type);
					UI_LOG_NET(QString("[%1] ← %2").arg(addr, cmdType.isEmpty() ? "(unknown)" : cmdType));

					if (m_dispatcher && !cmd.type.empty())
					{
						m_dispatcher->dispatch(cmd);
					}
				}
				catch (const std::exception& e)
				{
					const QString err = QString("解析/分发异常: %1").arg(e.what());
					LOG_RECEIVED(err);
					UI_LOG_NET_ERR(QString("[%1] %2").arg(addr, err));
				}
				catch (...)
				{
					const QString err = "解析/分发未知异常";
					LOG_RECEIVED(err);
					UI_LOG_NET_ERR(QString("[%1] %2").arg(addr, err));
				}

				// 移除已处理部分，继续扫描剩余数据
				buf.remove(0, i + 1);
				i         = -1;
				jsonStart = -1;
			}
		}
	}

	// 如果 buf 已全部处理完，停止超时计时
	if (buf.isEmpty())
	{
		m_buffers[socket].timeoutTimer->stop();
	}
}

// ─────────────────────────────────────────────
//  私有：清理 socket 相关资源
// ─────────────────────────────────────────────

void CcTcpServer::cleanupSocket(QTcpSocket* socket)
{
	if (!m_buffers.contains(socket))
		return;

	SocketBuffer& sb = m_buffers[socket];
	if (sb.timeoutTimer)
	{
		sb.timeoutTimer->stop();
		sb.timeoutTimer->deleteLater();
	}
	m_buffers.remove(socket);
}
