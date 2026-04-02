#include "CcTcpServer.h"
#include "CommandDispatcher.h"
#include "PointCloudService.h"
#include "MachineProxy.h"
#include "Handlers.h"
#include <ccMainAppInterface.h>

// 示例初始化代码
void initTcpPlugin(ccMainAppInterface* app) {
    // 1. 创建服务和代理
    PointCloudService* pointCloudService = new PointCloudService(app);
    MachineProxy* machineProxy = new MachineProxy();
    
    // 2. 创建命令分发器
    CommandDispatcher* dispatcher = new CommandDispatcher(pointCloudService, machineProxy);
    
    // 3. 注册 handler
    dispatcher->registerHandler("load", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("filter", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("icp", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("camera", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("applyViewport", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("applyTransformation", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("segment", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("delete", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("fit", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("clearDB", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("subsample", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("merge", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("clone", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("acquirePcd", std::make_unique<PointCloudCommandHandler>(pointCloudService));
    dispatcher->registerHandler("machine", std::make_unique<MachineCommandHandler>(machineProxy));
    
    // 4. 创建并启动 TCP 服务器
    CcTcpServer* tcpServer = new CcTcpServer();
    tcpServer->setCommandDispatcher(dispatcher);
    tcpServer->startListening(52700);
    
    // 命令流转示例：
    // 1. TCP 客户端发送 JSON 命令：{"action": "load", "params": {"path": "test.pcd", "name": "TestCloud"}}
    // 2. CcTcpServer 接收 JSON 并调用 CommandParser::parse() 解析为 Command 对象
    // 3. CcTcpServer 调用 CommandDispatcher::dispatch() 分发命令
    // 4. CommandDispatcher 根据命令类型 "load" 找到对应的 PointCloudCommandHandler
    // 5. PointCloudCommandHandler 调用 PointCloudService::load() 执行加载操作
    // 6. PointCloudService 使用 QMetaObject::invokeMethod() 在 UI 线程执行操作
}
