// qTcpPlugin.cpp
#include "qTcpPlugin.h"

#include "CcTcpServer.h"
#include "CommandDispatcher.h"
#include "PointCloudService.h"
#include "MachineProxy.h"
#include "Handlers.h"
#include "CommLogger.h"
#include <ccMainAppInterface.h>
#include <QCoreApplication>
#include <memory>

qTcpPlugin::qTcpPlugin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/qTcpPlugin/info.json")
    , m_startAction(nullptr)
    , m_stopAction(nullptr)
    , m_server(nullptr)
    , m_dispatcher(nullptr)
    , m_pointCloudService(nullptr)
    , m_machineProxy(nullptr)
{
}

qTcpPlugin::~qTcpPlugin()
{
}

void qTcpPlugin::onNewSelection(const ccHObject::Container&)
{
}
void qTcpPlugin::setMainAppInterface(ccMainAppInterface* app)
{
	ccStdPluginInterface::setMainAppInterface(app);
	startServer();

	// 插件初始化时调用一次
	CommLogger::instance().init(QCoreApplication::applicationDirPath());
}

QList<QAction*> qTcpPlugin::getActions()
{
	if (!m_startAction)
	{
		m_startAction = new QAction("启动 TCP 服务", this);
		connect(m_startAction, &QAction::triggered, this, &qTcpPlugin::startServer);
	}

	if (!m_stopAction)
	{
		m_stopAction = new QAction("停止 TCP 服务", this);
		connect(m_stopAction, &QAction::triggered, this, &qTcpPlugin::stopServer);
	}

	updateActions();
	return {m_startAction, m_stopAction};
}

void qTcpPlugin::startServer()
{
	if (m_server)
	{
		m_app->dispToConsole("[TcpPlugin] Server is already running",
		                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return;
	}

	// 1. 创建服务和代理
	m_pointCloudService = new PointCloudService(m_app, this);
	m_machineProxy = new MachineProxy(this);
	
	// 2. 创建命令分发器
	m_dispatcher = new CommandDispatcher(m_pointCloudService, m_machineProxy, this);
	
	// 3. 注册 handler
	m_dispatcher->registerHandler("load", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("filter", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("icp", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("camera", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("applyViewport", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("applyTransformation", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("segment", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("delete", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("fit", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("clearDB", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("subsample", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("merge", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("clone", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("acquirePcd", std::make_unique<PointCloudCommandHandler>(m_pointCloudService));
	m_dispatcher->registerHandler("machine", std::make_unique<MachineCommandHandler>(m_machineProxy));
	
	// 4. 创建并启动 TCP 服务器
	m_server = new CcTcpServer(this);
	m_server->setCommandDispatcher(m_dispatcher);

	quint16 port = 52700;
	if (m_server->startListening(port))
	{
		m_app->dispToConsole(QString("[TcpPlugin] Listening on port %1").arg(port));
	}
	else
	{
		m_app->dispToConsole("[TcpPlugin] Failed to start server",
		                     ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		delete m_server;
		m_server = nullptr;
		delete m_dispatcher;
		m_dispatcher = nullptr;
		delete m_pointCloudService;
		m_pointCloudService = nullptr;
		delete m_machineProxy;
		m_machineProxy = nullptr;
	}

	updateActions();
}

void qTcpPlugin::stopServer()
{
	if (!m_server)
	{
		m_app->dispToConsole("[TcpPlugin] Server is not running",
		                     ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		return;
	}

	m_server->close();
	delete m_server;
	m_server = nullptr;

	delete m_dispatcher;
	m_dispatcher = nullptr;

	delete m_pointCloudService;
	m_pointCloudService = nullptr;

	delete m_machineProxy;
	m_machineProxy = nullptr;

	m_app->dispToConsole("[TcpPlugin] Server stopped");
	updateActions();
}

void qTcpPlugin::updateActions()
{
	bool running = (m_server != nullptr);
	if (m_startAction)
		m_startAction->setEnabled(!running);
	if (m_stopAction)
		m_stopAction->setEnabled(running);
}
