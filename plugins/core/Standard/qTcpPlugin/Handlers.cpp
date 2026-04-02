#include "Handlers.h"

PointCloudCommandHandler::PointCloudCommandHandler(PointCloudService* service) : m_service(service) {
}

void PointCloudCommandHandler::handle(const Command& cmd) {
    if (cmd.type == "load") {
        m_service->load(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "filter") {
        m_service->filter(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "icp") {
        m_service->icp(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "camera") {
        m_service->camera(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "applyViewport") {
        m_service->applyViewport(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "applyTransformation") {
        m_service->applyTransformation(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "segment") {
        m_service->segment(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "delete") {
        m_service->deleteObject(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "fit") {
        m_service->fit(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "clearDB") {
        m_service->clearDB(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "subsample") {
        m_service->subsample(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "merge") {
        m_service->merge(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "clone") {
        m_service->clone(cmd.params, cmd.socket, cmd.idCode);
    } else if (cmd.type == "acquirePcd") {
        m_service->acquirePcd(cmd.params, cmd.socket, cmd.idCode);
    }
}

MachineCommandHandler::MachineCommandHandler(MachineProxy* proxy) : m_proxy(proxy) {
}

void MachineCommandHandler::handle(const Command& cmd) {
    m_proxy->send(cmd);
}
