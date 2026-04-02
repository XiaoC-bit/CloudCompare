#include "TcpClient.h"
#include <QJsonDocument>

TcpClient::TcpClient(QObject* parent) : QObject(parent) {
    m_socket = new QTcpSocket(this);
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
