// LogDockWidget.h
#pragma once
#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QDockWidget>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QScrollBar>
#include <QTextBlock>
#include <QTextEdit>

// ── 内部子类：重写双击选整行、右键菜单、Ctrl+A ───────────
class LogTextEdit : public QTextEdit
{
	Q_OBJECT
  public:
	explicit LogTextEdit(QWidget* parent = nullptr) : QTextEdit(parent)
	{
		setContextMenuPolicy(Qt::CustomContextMenu);
		connect(this, &QWidget::customContextMenuRequested, this, &LogTextEdit::showContextMenu);
	}

  protected:
	// 双击 → 选中整行（一条日志）
	void mouseDoubleClickEvent(QMouseEvent* e) override
	{
		QTextCursor cursor = cursorForPosition(e->pos());
		cursor.movePosition(QTextCursor::StartOfBlock);
		cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
		setTextCursor(cursor);
		e->accept();
	}

	// Ctrl+A 全选
	void keyPressEvent(QKeyEvent* e) override
	{
		if (e->key() == Qt::Key_A && (e->modifiers() & Qt::ControlModifier))
		{
			selectAll();
			e->accept();
			return;
		}
		QTextEdit::keyPressEvent(e);
	}

  private slots:
	void showContextMenu(const QPoint& pos)
	{
		// 取右键点击位置所在行的纯文本
		QTextCursor lineCursor = cursorForPosition(pos);
		lineCursor.movePosition(QTextCursor::StartOfBlock);
		lineCursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
		const QString lineText = lineCursor.selectedText();
		const bool    hasLine  = !lineText.trimmed().isEmpty();
		const bool    hasAny   = !document()->isEmpty();

		QMenu menu(this);

		QAction* actCopyLine = menu.addAction("复制此行");
		actCopyLine->setEnabled(hasLine);

		QAction* actCopyAll = menu.addAction("复制全部");
		actCopyAll->setEnabled(hasAny);

		menu.addSeparator();

		QAction* actSelectAll = menu.addAction("全选\tCtrl+A");
		actSelectAll->setEnabled(hasAny);

		menu.addSeparator();

		QAction* actClear = menu.addAction("清空日志");
		actClear->setEnabled(hasAny);

		QAction* chosen = menu.exec(viewport()->mapToGlobal(pos));
		if (!chosen)
			return;

		if (chosen == actCopyLine)
			QApplication::clipboard()->setText(lineText);
		else if (chosen == actCopyAll)
			QApplication::clipboard()->setText(toPlainText());
		else if (chosen == actSelectAll)
			selectAll();
		else if (chosen == actClear)
			clear();
	}
};

// ── 主窗口 ────────────────────────────────────────────────
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
		m_textEdit = new LogTextEdit(this);
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

			    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");

			    QString levelTag;
			    QString levelColor;
			    switch (level)
			    {
				    case Level::Info:
					    levelTag   = "INFO ";
					    levelColor = "#4ec9b0";
					    break;
				    case Level::Warning:
					    levelTag   = "WARN ";
					    levelColor = "#dcdcaa";
					    break;
				    case Level::Error:
					    levelTag   = "ERROR";
					    levelColor = "#f44747";
					    break;
			    }

			    QString categoryTag;
			    QString categoryColor;
			    switch (category)
			    {
				    case Category::Network:
					    categoryTag   = "NET ";
					    categoryColor = "#569cd6";
					    break;
				    case Category::Plugin:
					    categoryTag   = "PLUG";
					    categoryColor = "#c586c0";
					    break;
			    }

			    const QString html = QString(
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

	LogTextEdit* m_textEdit;
};
