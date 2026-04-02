#include "MachineProxy.h"
#include "TcpClient.h"

MachineProxy::MachineProxy(QObject* parent) : QObject(parent) {
    m_client = new TcpClient(this);
    // 这里可以添加默认的连接逻辑
    // m_client->connectToHost("localhost", 52701);
}

void MachineProxy::send(const Command& cmd) {
    // 将 Command 转换为机床协议的 JSON
    QJsonObject machineCmd;
    machineCmd["type"] = QString::fromStdString(cmd.type);
    machineCmd["params"] = cmd.params;
    
    // 发送到机床控制中间件
    m_client->send(machineCmd);
}
