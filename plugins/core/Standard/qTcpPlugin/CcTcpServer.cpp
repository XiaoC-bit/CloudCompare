#include "CcTcpServer.h"
#include "CommandParser.h"
#include "CommandDispatcher.h"
#include "CommLogger.h"

CcTcpServer::CcTcpServer(QObject* parent) : QTcpServer(parent) {
    m_parser = new CommandParser();
    m_dispatcher = nullptr;
}

bool CcTcpServer::startListening(quint16 port) {
    return listen(QHostAddress::Any, port);
}

void CcTcpServer::setCommandDispatcher(CommandDispatcher* dispatcher) {
    m_dispatcher = dispatcher;
}

void CcTcpServer::incomingConnection(qintptr socketDescriptor) {
    QTcpSocket* socket = new QTcpSocket(this);
    socket->setSocketDescriptor(socketDescriptor);
    connect(socket, &QTcpSocket::readyRead, this, &CcTcpServer::onReadyRead);
    connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
}

void CcTcpServer::onReadyRead() {
    QTcpSocket* socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) {
        return;
    }

    m_buffer.append(socket->readAll());
    
    // 简单的 JSON 解析，假设每个命令是一个完整的 JSON 对象
    if (m_buffer.contains('{') && m_buffer.contains('}')) {
        QString jsonStr = QString::fromUtf8(m_buffer);
        LOG_RECEIVED(jsonStr);
        Command cmd = m_parser->parse(jsonStr);
        cmd.socket = socket;
        
        if (m_dispatcher && !cmd.type.empty()) {
            m_dispatcher->dispatch(cmd);
        }
        
        m_buffer.clear();
    }
}
