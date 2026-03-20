// CommandDispatcher.cpp
#include "CommandDispatcher.h"

#include <FileIOFilter.h>
#include <ccMainAppInterface.h>
#include <ccGLWindowInterface.h>
#include <ccPointCloud.h>
#include <QFileInfo>

CommandDispatcher::CommandDispatcher(ccMainAppInterface* app, QObject* parent)
    : QObject(parent)
    , m_app(app)
{
}

void CommandDispatcher::dispatch(QJsonObject cmd)
{
	QString action = cmd["action"].toString();

	if (action == "load")
		handleLoad(cmd["params"].toObject());
	else if (action == "filter")
		handleFilter(cmd["params"].toObject());
	else if (action == "icp")
		handleICP(cmd["params"].toObject());
	else if (action == "camera")
		handleCamera(cmd["params"].toObject());
	else
		m_app->dispToConsole("[TcpPlugin] Unknown action: " + action, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
}

void CommandDispatcher::handleLoad(const QJsonObject& params)
{
	QString path = params["path"].toString();
	if (path.isEmpty())
		return;

	// 使用 CC 的文件加载机制
	CC_FILE_ERROR result;
	auto          filter = FileIOFilter::FindBestFilterForExtension(QFileInfo(path).suffix());
	if (!filter)
	{
		m_app->dispToConsole("[TcpPlugin] Unsupported file format", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* container = new ccHObject();
	auto result = filter->loadFile(path, *container, FileIOFilter::LoadParameters());
	if (result == CC_FERR_NO_ERROR)
	{
		m_app->addToDB(container);
		m_app->dispToConsole("[TcpPlugin] Loaded: " + path);
	}
}

void CommandDispatcher::handleCamera(const QJsonObject& params)
{
	/*auto* glWindow = m_app->getActiveGLWindow();
	if (!glWindow)
		return;

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
	}*/
}

void CommandDispatcher::handleFilter(const QJsonObject& params)
{
}
void CommandDispatcher::handleICP(const QJsonObject& params)
{
}
