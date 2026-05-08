// LogDockWidget.h
#pragma once
#include <QDateTime>
#include <QDockWidget>
#include <QTextEdit>
#include <QTextBlock>
#include <QScrollBar>

class LogDockWidget : public QDockWidget
{
	Q_OBJECT

  public:
	enum class Level
	{
		Info,
		Warning,
		Error
	};

	enum class Category
	{
		Network, // 通讯
		Plugin   // 插件内部
	};

	static constexpr int kMaxLines = 2000;

	explicit LogDockWidget(QWidget* parent = nullptr)
	    : QDockWidget("通讯日志", parent)
	{
		m_textEdit = new QTextEdit(this);
		m_textEdit->setReadOnly(true);
		m_textEdit->setFont(QFont("Consolas", 9));
		m_textEdit->setStyleSheet(
		    "QTextEdit {"
		    "  background-color: #1e1e1e;"
		    "  color: #d4d4d4;"
		    "  border: none;"
		    "}");
		setWidget(m_textEdit);
		setMinimumWidth(500);
	}

	// 线程安全：可从任意线程调用
	void appendLog(const QString& message,
	               Level          level    = Level::Info,
	               Category       category = Category::Network)
	{
		QMetaObject::invokeMethod(
		    this,
		    [=]()
		    {
			    trimToMaxLines();

			    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");

			    // 等级标签
			    QString levelTag;
			    QString levelColor;
			    switch (level)
			    {
				    case Level::Info:
					    levelTag   = "INFO ";
					    levelColor = "#4ec9b0"; // 青绿
					    break;
				    case Level::Warning:
					    levelTag   = "WARN ";
					    levelColor = "#dcdcaa"; // 黄
					    break;
				    case Level::Error:
					    levelTag   = "ERROR";
					    levelColor = "#f44747"; // 红
					    break;
			    }

			    // 类型标签
			    QString categoryTag;
			    QString categoryColor;
			    switch (category)
			    {
				    case Category::Network:
					    categoryTag   = "NET ";
					    categoryColor = "#569cd6"; // 蓝
					    break;
				    case Category::Plugin:
					    categoryTag   = "PLUG";
					    categoryColor = "#c586c0"; // 紫
					    break;
			    }

			    // 拼接 HTML 行，各字段用固定宽度 span 对齐
			    QString html = QString(
			                       "<span style='color:#6a9955;'>%1</span> "
			                       "<span style='color:%2; font-weight:bold;'>[%3]</span> "
			                       "<span style='color:%4;'>[%5]</span> "
			                       "<span style='color:#d4d4d4;'>%6</span>")
			                       .arg(timestamp.toHtmlEscaped(),
			                            levelColor,
			                            levelTag.toHtmlEscaped(),
			                            categoryColor,
			                            categoryTag.toHtmlEscaped(),
			                            message.toHtmlEscaped());

			    m_textEdit->append(html);

			    // 自动滚动到最新行
			    m_textEdit->verticalScrollBar()->setValue(
			        m_textEdit->verticalScrollBar()->maximum());
		    },
		    Qt::QueuedConnection);
	}

  public slots:
	void clearLog()
	{
		m_textEdit->clear();
	}

  private:
	// 超出上限时删除最旧的行，保持内存稳定
	void trimToMaxLines()
	{
		QTextDocument* doc = m_textEdit->document();
		while (doc->blockCount() > kMaxLines)
		{
			QTextCursor cursor(doc->begin());
			cursor.select(QTextCursor::BlockUnderCursor);
			cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor);
			cursor.removeSelectedText();
		}
	}

	QTextEdit* m_textEdit;
};
