// qTcpPlugin.cpp
#include "qTcpPlugin.h"

#include "CcTcpServer.h"
#include "CommandDispatcher.h"

#include <ccMainAppInterface.h>

qTcpPlugin::qTcpPlugin(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/qTcpPlugin/info.json")
    , m_startAction(nullptr)
    , m_stopAction(nullptr)
    , m_server(nullptr)
    , m_dispatcher(nullptr)
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

	m_server     = new CcTcpServer(this);
	m_dispatcher = new CommandDispatcher(m_app, m_server, this);

	connect(m_server, &CcTcpServer::commandReceived, m_dispatcher, &CommandDispatcher::dispatch, Qt::QueuedConnection);

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
