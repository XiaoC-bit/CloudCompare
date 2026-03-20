// CommandDispatcher.h
#pragma once
#include <QJsonObject>
#include <QObject>

class ccMainAppInterface;

class CommandDispatcher : public QObject
{
	Q_OBJECT
  public:
	explicit CommandDispatcher(ccMainAppInterface* app, QObject* parent = nullptr);

  public slots:
	void dispatch(QJsonObject cmd);

  private:
	ccMainAppInterface* m_app;

	void handleLoad(const QJsonObject& params);
	void handleFilter(const QJsonObject& params);
	void handleICP(const QJsonObject& params);
	void handleCamera(const QJsonObject& params);								    
};
