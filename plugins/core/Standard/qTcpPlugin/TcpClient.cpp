#include "TcpClient.h"
#include <QJsonDocument>

TcpClient::TcpClient(QObject* parent) : QObject(parent) {
    m_socket = new QTcpSocket(this);
    connect(m_socket, &QTcpSocket::connected, this, &TcpClient::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &TcpClient::onDisconnected);
    connect(m_socket, &QTcpSocket::readyRead, this, &TcpClient::onReadyRead);
}

void TcpClient::connectToHost(const QString& host, quint16 port) {
    m_socket->connectToHost(host, port);
}

void TcpClient::send(const QJsonObject& data) {
    QJsonDocument doc(data);
    QByteArray jsonData = doc.toJson(QJsonDocument::Compact);
    m_socket->write(jsonData);
    m_socket->flush();
}

bool TcpClient::isConnected() const {
    return m_socket->state() == QTcpSocket::ConnectedState;
}

void TcpClient::onConnected() {
    emit connected();
}

void TcpClient::onDisconnected() {
    emit disconnected();
}

void TcpClient::onError(QAbstractSocket::SocketError socketError) {
    emit errorOccurred(socketError);
}

void TcpClient::onReadyRead() {
    m_buffer.append(m_socket->readAll());
    
    // 简单的 JSON 解析，假设每个响应是一个完整的 JSON 对象
    if (m_buffer.contains('{') && m_buffer.contains('}')) {
        QJsonDocument doc = QJsonDocument::fromJson(m_buffer);
        if (doc.isObject()) {
            emit responseReceived(doc.object());
        }
        m_buffer.clear();
    }
}
