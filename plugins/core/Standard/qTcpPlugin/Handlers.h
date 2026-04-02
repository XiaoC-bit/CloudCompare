#pragma once
#include "ICommandHandler.h"
#include "PointCloudService.h"
#include "MachineProxy.h"

class PointCloudCommandHandler : public ICommandHandler {
public:
    explicit PointCloudCommandHandler(PointCloudService* service);
    void handle(const Command& cmd) override;

private:
    PointCloudService* m_service;
};

class MachineCommandHandler : public ICommandHandler {
public:
    explicit MachineCommandHandler(MachineProxy* proxy);
    void handle(const Command& cmd) override;

private:
    MachineProxy* m_proxy;
};
