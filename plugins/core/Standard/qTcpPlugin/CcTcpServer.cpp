// CcTcpServer.cpp
#include "CcTcpServer.h"

#include <QJsonDocument>
#include <QThreadPool>

CcTcpServer::CcTcpServer(QObject* parent)
    : QTcpServer(parent)
{
}

bool CcTcpServer::startListening(quint16 port)
{
	return listen(QHostAddress::LocalHost, port);
}

void CcTcpServer::incomingConnection(qintptr socketDescriptor)
{
	// 每个连接在主线程处理（消息量小，无需多线程）
	auto* socket = new QTcpSocket(this);
	socket->setSocketDescriptor(socketDescriptor);
	connect(socket, &QTcpSocket::readyRead, this, &CcTcpServer::onReadyRead);
	connect(socket, &QTcpSocket::disconnected, socket, &QObject::deleteLater);
	addPendingConnection(socket);
}

void CcTcpServer::onReadyRead()
{
	m_buffer.clear();
	auto* socket = qobject_cast<QTcpSocket*>(sender());
	m_buffer += socket->readAll();

	QJsonParseError err;
	auto            doc = QJsonDocument::fromJson(m_buffer, &err);
	if (err.error != QJsonParseError::NoError)
	{
		// 数据可能还没收完，等下一次 readyRead
		return;
	}

	// 解析成功，清空缓冲区，发射信号（包含socket指针）
	m_buffer.clear();
	emit commandReceived(doc.object(), socket);
}

void CcTcpServer::sendResponse(QTcpSocket* socket, bool ok, const QString& msg)
{
	QJsonObject resp;
	resp["ok"]  = ok;
	resp["msg"] = msg;
	socket->write(QJsonDocument(resp).toJson(QJsonDocument::Compact) + "\n");
	socket->flush();
}
