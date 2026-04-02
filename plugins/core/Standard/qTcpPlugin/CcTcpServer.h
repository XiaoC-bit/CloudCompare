#pragma once
#include <QJsonObject>
#include <QTcpServer>
#include <QTcpSocket>

class CommandParser;
class CommandDispatcher;

class CcTcpServer : public QTcpServer {
    Q_OBJECT
public:
    explicit CcTcpServer(QObject* parent = nullptr);
    bool startListening(quint16 port = 52700);
    void setCommandDispatcher(CommandDispatcher* dispatcher);

protected:
    void incomingConnection(qintptr socketDescriptor) override;

private slots:
    void onReadyRead();

private:
    QByteArray m_buffer;
    CommandParser* m_parser;
    CommandDispatcher* m_dispatcher;
};
