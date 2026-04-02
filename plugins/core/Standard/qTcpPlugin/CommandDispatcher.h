#pragma once
#include <QObject>
#include <unordered_map>
#include <memory>
#include "Command.h"
#include "ICommandHandler.h"

class PointCloudService;
class MachineProxy;

class CommandDispatcher : public QObject {
    Q_OBJECT
public:
    explicit CommandDispatcher(PointCloudService* pointCloudService, MachineProxy* machineProxy, QObject* parent = nullptr);
    void dispatch(const Command& cmd);
    void registerHandler(const std::string& commandType, std::shared_ptr<ICommandHandler> handler);

private:
	std::unordered_map<std::string, std::shared_ptr<ICommandHandler>> m_handlers;
    PointCloudService* m_pointCloudService;
    MachineProxy* m_machineProxy;
};
