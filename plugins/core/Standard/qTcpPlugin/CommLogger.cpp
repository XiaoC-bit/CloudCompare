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
	if (m_commFile.isOpen())
		m_commFile.close();
	if (m_internalFile.isOpen())
		m_internalFile.close();
}

void CommLogger::ensureCommFileOpen()
{
	QDate today = QDate::currentDate();

	// 日期变了就关掉旧文件，开新文件
	if (m_commFile.isOpen() && m_currentDate != today)
	{
		m_commFile.close();
	}

	if (!m_commFile.isOpen())
	{
		m_currentDate    = today;
		QString fileName = m_logDir + "/" + today.toString("yyyy-MM-dd") + "_comm.log";
		m_commFile.setFileName(fileName);
		m_commFile.open(QIODevice::Append | QIODevice::Text);
		m_commStream.setDevice(&m_commFile);
	}
}

void CommLogger::ensureInternalFileOpen()
{
	QDate today = QDate::currentDate();

	// 日期变了就关掉旧文件，开新文件
	if (m_internalFile.isOpen() && m_currentDate != today)
	{
		m_internalFile.close();
	}

	if (!m_internalFile.isOpen())
	{
		m_currentDate    = today;
		QString fileName = m_logDir + "/" + today.toString("yyyy-MM-dd") + "_internal.log";
		m_internalFile.setFileName(fileName);
		m_internalFile.open(QIODevice::Append | QIODevice::Text);
		m_internalStream.setDevice(&m_internalFile);
	}
}

void CommLogger::writeLine(QFile& file, QTextStream& stream, QMutex& mutex, const QString& tag, const QString& content)
{
	if (m_logDir.isEmpty())
		return;

	QMutexLocker locker(&mutex);

	if (!file.isOpen())
		return;

	QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
	stream << QString("[%1][%2] %3\n").arg(timestamp, tag, content);
	stream.flush();
}

void CommLogger::logReceived(const QString& content)
{
	ensureCommFileOpen();
	writeLine(m_commFile, m_commStream, m_commMutex, "RECV", content);
}

void CommLogger::logSent(const QString& content)
{
	ensureCommFileOpen();
	writeLine(m_commFile, m_commStream, m_commMutex, "SEND", content);
}

void CommLogger::logInternal(const QString& content)
{
	ensureInternalFileOpen();
	writeLine(m_internalFile, m_internalStream, m_internalMutex, "INTERNAL", content);
}
