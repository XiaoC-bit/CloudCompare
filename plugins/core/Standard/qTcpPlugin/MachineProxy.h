#pragma once
#include <QObject>
#include "Command.h"

class TcpClient;

class MachineProxy : public QObject {
    Q_OBJECT
public:
    explicit MachineProxy(QObject* parent = nullptr);
    void send(const Command& cmd);

private:
    TcpClient* m_client;
};
