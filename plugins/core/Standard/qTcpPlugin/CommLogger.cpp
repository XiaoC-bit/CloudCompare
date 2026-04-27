#include "CommLogger.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>

void CommLogger::init(const QString& exeDir, qint64 maxFileSize)
{
	m_logDir = exeDir + "/log";
	QDir().mkpath(m_logDir);
	
	// 设置日志文件大小限制
	m_maxFileSize = maxFileSize;
}

CommLogger::~CommLogger()
{
	if (m_commFile.isOpen())
		m_commFile.close();
	if (m_internalFile.isOpen())
		m_internalFile.close();
}

bool CommLogger::shouldRollFile(const QFile& file) const
{
	if (!file.isOpen())
		return false;
	return file.size() > m_maxFileSize;
}

QString CommLogger::getRolledFileName(const QString& baseName) const
{
	QString rolledName;
	int index = 1;
	
	// 生成带序号的文件名，直到找到一个不存在的
	while (true)
	{
		rolledName = baseName + "." + QString::number(index) + ".log";
		if (!QFile::exists(rolledName))
			break;
		index++;
	}
	
	return rolledName;
}

void CommLogger::ensureCommFileOpen()
{
	QDate today = QDate::currentDate();

	// 日期变了就关掉旧文件，开新文件
	if (m_commFile.isOpen() && m_currentDate != today)
	{
		m_commFile.close();
	}

	// 检查文件大小，如果超过限制就滚动
	if (m_commFile.isOpen() && shouldRollFile(m_commFile))
	{
		QString oldName = m_commFile.fileName();
		m_commFile.close();
		
		// 生成滚动后的文件名
		QString rolledName = getRolledFileName(oldName.left(oldName.length() - 4));
		QFile::rename(oldName, rolledName);
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

	// 检查文件大小，如果超过限制就滚动
	if (m_internalFile.isOpen() && shouldRollFile(m_internalFile))
	{
		QString oldName = m_internalFile.fileName();
		m_internalFile.close();
		
		// 生成滚动后的文件名
		QString rolledName = getRolledFileName(oldName.left(oldName.length() - 4));
		QFile::rename(oldName, rolledName);
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
