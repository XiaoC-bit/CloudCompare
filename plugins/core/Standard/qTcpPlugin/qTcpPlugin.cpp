// qTcpPlugin.cpp
#include "qTcpPlugin.h"

#include "CcTcpServer.h"
#include "CommandDispatcher.h"
#include "PointCloudService.h"
#include "MachineProxy.h"
#include "Handlers.h"
#include "CommLogger.h"
#include "CalibrationDialog.h"
#include <ccMainAppInterface.h>
#include <QCoreApplication>
#include <memory>

qTcpPlugin::qTcpPlugin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/qTcpPlugin/info.json")
    , m_startAction(nullptr)
    , m_stopAction(nullptr)
    , m_calibrationAction(nullptr)
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

	if (!m_calibrationAction)
	{
		m_calibrationAction = new QAction("标定", this);
		connect(m_calibrationAction, &QAction::triggered, this, &qTcpPlugin::showCalibrationDialog);
	}

	updateActions();
	return {m_startAction, m_stopAction, m_calibrationAction};
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
	auto handler = std::make_shared<PointCloudCommandHandler>(m_pointCloudService);
	m_dispatcher->registerHandler("load", handler);
	m_dispatcher->registerHandler("filter", handler);
	m_dispatcher->registerHandler("icp", handler);
	m_dispatcher->registerHandler("camera", handler);
	m_dispatcher->registerHandler("applyViewport", handler);
	m_dispatcher->registerHandler("applyTransformation", handler);
	m_dispatcher->registerHandler("segment", handler);
	m_dispatcher->registerHandler("delete", handler);
	m_dispatcher->registerHandler("fit", handler);
	m_dispatcher->registerHandler("clearDB", handler);
	m_dispatcher->registerHandler("subsample", handler);
	m_dispatcher->registerHandler("merge", handler);
	m_dispatcher->registerHandler("clone", handler);
	m_dispatcher->registerHandler("acquirePcd", handler);
	m_dispatcher->registerHandler("startCalibration", handler);

	auto machineHandler = std::make_shared<MachineCommandHandler>(m_machineProxy);
	m_dispatcher->registerHandler("machine", machineHandler);
	
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

void qTcpPlugin::showCalibrationDialog()
{
	CalibrationDialog dialog(m_app,m_pointCloudService, nullptr);
	if (dialog.exec() == QDialog::Accepted)
	{
		QVector<QVector3D> positions = dialog.getPositions();
		m_app->dispToConsole(QString("[TcpPlugin] 标定完成，共 %1 个位置").arg(positions.size()));
	}
}
