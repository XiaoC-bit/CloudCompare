#pragma once
#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>
#include <QTimer>

class TcpClient : public QObject {
    Q_OBJECT
public:
    explicit TcpClient(QObject* parent = nullptr);
    void connectToHost(const QString& host, quint16 port);
    void send(const QJsonObject& data);
    bool isConnected() const;

signals:
    void connected();
    void disconnected();
    void errorOccurred(QAbstractSocket::SocketError socketError);
    void responseReceived(const QJsonObject& response);

private slots:
    void onConnected();
    void onDisconnected();
    void onError(QAbstractSocket::SocketError socketError);
    void onReadyRead();

private:
    QTcpSocket* m_socket;
    QByteArray m_buffer;
};
