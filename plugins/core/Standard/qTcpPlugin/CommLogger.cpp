#include "CommLogger.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>

void CommLogger::init(const QString& exeDir)
{
	m_logDir = exeDir + "/log";
	QDir().mkpath(m_logDir);
}

CommLogger::~CommLogger()
{
	if (m_file.isOpen())
		m_file.close();
}

void CommLogger::ensureFileOpen()
{
	QDate today = QDate::currentDate();

	// 日期变了就关掉旧文件，开新文件
	if (m_file.isOpen() && m_currentDate != today)
	{
		m_file.close();
	}

	if (!m_file.isOpen())
	{
		m_currentDate    = today;
		QString fileName = m_logDir + "/" + today.toString("yyyy-MM-dd") + ".log";
		m_file.setFileName(fileName);
		m_file.open(QIODevice::Append | QIODevice::Text);
		m_stream.setDevice(&m_file);
	}
}

void CommLogger::writeLine(const QString& tag, const QString& content)
{
	if (m_logDir.isEmpty())
		return;

	QMutexLocker locker(&m_mutex);
	ensureFileOpen();

	if (!m_file.isOpen())
		return;

	QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
	m_stream << QString("[%1][%2] %3\n").arg(timestamp, tag, content);
	m_stream.flush();
}

void CommLogger::logReceived(const QString& content)
{
	writeLine("RECV", content);
}

void CommLogger::logSent(const QString& content)
{
	writeLine("SEND", content);
}
