// CommandDispatcher.cpp
#include "CommandDispatcher.h"
#include "CcTcpServer.h"

#include <FileIOFilter.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <QFileInfo>

CommandDispatcher::CommandDispatcher(ccMainAppInterface* app, CcTcpServer* server, QObject* parent)
    : QObject(parent)
    , m_app(app)
    , m_server(server)
{
}

void CommandDispatcher::dispatch(QJsonObject cmd, QTcpSocket* socket)
{
	QString action = cmd["action"].toString();

	if (action == "load")
		handleLoad(cmd["params"].toObject(), socket);
	else if (action == "filter")
		handleFilter(cmd["params"].toObject(), socket);
	else if (action == "icp")
		handleICP(cmd["params"].toObject(), socket);
	else if (action == "camera")
		handleCamera(cmd["params"].toObject(), socket);
	else
	{
		m_app->dispToConsole("[TcpPlugin] Unknown action: " + action, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Unknown action: " + action);
	}
}

void CommandDispatcher::handleLoad(const QJsonObject& params, QTcpSocket* socket)
{
	QString path = params["path"].toString();
	if (path.isEmpty())
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Empty path");
		return;
	}

	auto filter = FileIOFilter::FindBestFilterForExtension(QFileInfo(path).suffix());
	if (!filter)
	{
		m_app->dispToConsole("[TcpPlugin] Unsupported file format", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Unsupported file format");
		return;
	}
	
	ccHObject* container = new ccHObject();
	auto result = filter->loadFile(path, *container, FileIOFilter::LoadParameters());
	if (result == CC_FERR_NO_ERROR)
	{
		m_app->addToDB(container);
		m_app->dispToConsole("[TcpPlugin] Loaded: " + path);
		if (m_server)
			m_server->sendResponse(socket, true, "Loaded: " + path);
	}
	else
	{
		m_app->dispToConsole("[TcpPlugin] Failed to load: " + path, ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		if (m_server)
			m_server->sendResponse(socket, false, "Failed to load: " + path);
		delete container;
	}
}

void CommandDispatcher::handleCamera(const QJsonObject& params, QTcpSocket* socket)
{
	auto* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
	{
		if (m_server)
			m_server->sendResponse(socket, false, "No active GL window");
		return;
	}

	if (params.contains("viewPreset"))
	{
		QString preset = params["viewPreset"].toString();
		if (preset == "top")
			glWindow->setView(CC_TOP_VIEW);
		else if (preset == "front")
			glWindow->setView(CC_FRONT_VIEW);
		else if (preset == "iso")
			glWindow->setView(CC_ISO_VIEW_1);
		glWindow->redraw();
		if (m_server)
			m_server->sendResponse(socket, true, "Camera view set to: " + preset);
	}
	else
	{
		if (m_server)
			m_server->sendResponse(socket, false, "Missing viewPreset parameter");
	}
}

void CommandDispatcher::handleFilter(const QJsonObject& params, QTcpSocket* socket)
{
	if (m_server)
		m_server->sendResponse(socket, false, "Filter not implemented");
}

void CommandDispatcher::handleICP(const QJsonObject& params, QTcpSocket* socket)
{
	if (m_server)
		m_server->sendResponse(socket, false, "ICP not implemented");
}
