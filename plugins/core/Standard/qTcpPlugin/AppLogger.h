// AppLogger.h
// UI日志窗口的全局访问点，与 CommLogger（文件日志）配合使用。
// 用法：
//   初始化（主线程）：AppLogger::instance().setWidget(m_logDock);
//   记录日志（任意线程）：
//     UI_LOG_NET("收到消息: " + json);
//     UI_LOG_NET_WARN("连接超时");
//     UI_LOG_NET_ERR("解析失败");
//     UI_LOG_PLUG("ICP 配准完成");
//     UI_LOG_PLUG_WARN("点云为空，跳过");
//     UI_LOG_PLUG_ERR("加载失败: " + err);
#pragma once
#include "LogDockWidget.h"

class AppLogger
{
  public:
	static AppLogger& instance()
	{
		static AppLogger s;
		return s;
	}

	void setWidget(LogDockWidget* widget) { m_widget = widget; }

	void log(const QString&        message,
	         LogDockWidget::Level    level    = LogDockWidget::Level::Info,
	         LogDockWidget::Category category = LogDockWidget::Category::Network)
	{
		if (m_widget)
			m_widget->appendLog(message, level, category);
	}

  private:
	AppLogger() = default;
	LogDockWidget* m_widget = nullptr;
};

// ── 通讯类日志 ──────────────────────────────────────────
#define UI_LOG_NET(msg)      AppLogger::instance().log(msg, LogDockWidget::Level::Info,    LogDockWidget::Category::Network)
#define UI_LOG_NET_WARN(msg) AppLogger::instance().log(msg, LogDockWidget::Level::Warning, LogDockWidget::Category::Network)
#define UI_LOG_NET_ERR(msg)  AppLogger::instance().log(msg, LogDockWidget::Level::Error,   LogDockWidget::Category::Network)

// ── 插件内部日志 ────────────────────────────────────────
#define UI_LOG_PLUG(msg)      AppLogger::instance().log(msg, LogDockWidget::Level::Info,    LogDockWidget::Category::Plugin)
#define UI_LOG_PLUG_WARN(msg) AppLogger::instance().log(msg, LogDockWidget::Level::Warning, LogDockWidget::Category::Plugin)
#define UI_LOG_PLUG_ERR(msg)  AppLogger::instance().log(msg, LogDockWidget::Level::Error,   LogDockWidget::Category::Plugin)
