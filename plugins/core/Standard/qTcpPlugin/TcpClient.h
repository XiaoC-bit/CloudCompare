#pragma once
#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>

class TcpClient : public QObject {
    Q_OBJECT
public:
    explicit TcpClient(QObject* parent = nullptr);
    void connectToHost(const QString& host, quint16 port);
    void send(const QJsonObject& data);

private:
    QTcpSocket* m_socket;
};
