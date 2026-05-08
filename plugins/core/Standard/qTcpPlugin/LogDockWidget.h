// LogDockWidget.h
#pragma once
#include <QDateTime>
#include <QDockWidget>
#include <QTextEdit>

class LogDockWidget : public QDockWidget
{
	Q_OBJECT
  public:
	explicit LogDockWidget(QWidget* parent = nullptr)
	    : QDockWidget("通讯日志", parent)
	{
		m_textEdit = new QTextEdit(this);
		m_textEdit->setReadOnly(true);
		m_textEdit->setFont(QFont("Consolas", 9));
		setWidget(m_textEdit);
		setMinimumWidth(400);
	}

	// 线程安全：TCP线程调用此接口
	void appendLog(const QString& clientAddr, const QString& api, const QString& detail = "")
	{
		// 必须在主线程操作UI，用invokeMethod保证
		QMetaObject::invokeMethod(this, [=]()
		                          {
            QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
            QString line = QString("[%1] %2 → %3").arg(timestamp, clientAddr, api);
            if (!detail.isEmpty())
                line += "  " + detail;
            m_textEdit->append(line); },
		                          Qt::QueuedConnection);
	}

  public slots:
	void clearLog()
	{
		m_textEdit->clear();
	}

  private:
	QTextEdit* m_textEdit;
};
