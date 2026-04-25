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
	void logInternal(const QString& content);

  private:
	CommLogger() = default;
	~CommLogger();

	void ensureCommFileOpen();
	void ensureInternalFileOpen();
	void writeLine(QFile& file, QTextStream& stream, QMutex& mutex, const QString& tag, const QString& content);

	QFile       m_commFile;
	QTextStream m_commStream;
	QMutex      m_commMutex;
	QFile       m_internalFile;
	QTextStream m_internalStream;
	QMutex      m_internalMutex;
	QString     m_logDir;
	QDate       m_currentDate;
};
