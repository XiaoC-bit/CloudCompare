#pragma once
#include <QDate>
#include <QFile>
#include <QMutex>
#include <QObject>
#include <QTextStream>

class CommLogger
{
  public:
	static CommLogger& instance()
	{
		static CommLogger s_instance;
		return s_instance;
	}

	void init(const QString& exeDir);
	void logReceived(const QString& content);
	void logSent(const QString& content);

  private:
	CommLogger() = default;
	~CommLogger();

	void ensureFileOpen();
	void writeLine(const QString& tag, const QString& content);

	QFile       m_file;
	QTextStream m_stream;
	QMutex      m_mutex;
	QString     m_logDir;
	QDate       m_currentDate;
};
