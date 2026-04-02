#include "CommandDispatcher.h"
#include "PointCloudService.h"
#include "MachineProxy.h"

CommandDispatcher::CommandDispatcher(PointCloudService* pointCloudService, MachineProxy* machineProxy, QObject* parent) 
    : QObject(parent), m_pointCloudService(pointCloudService), m_machineProxy(machineProxy) {
}

void CommandDispatcher::dispatch(const Command& cmd) {
    auto it = m_handlers.find(cmd.type);
    if (it != m_handlers.end()) {
        it->second->handle(cmd);
    }
}

void CommandDispatcher::registerHandler(const std::string& commandType, std::shared_ptr<ICommandHandler> handler)
{
	m_handlers[commandType] = handler;
}
