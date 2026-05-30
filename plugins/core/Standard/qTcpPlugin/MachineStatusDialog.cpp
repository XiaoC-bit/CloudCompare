#include "MachineStatusDialog.h"
#include "PointCloudService.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFrame>
#include <ccMainAppInterface.h>
#include <QTimer>
#include <qevent.h>

MachineStatusDialog::MachineStatusDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
    : QDialog(parent)
    , m_pointCloudService(pointCloudService)
    , m_app(app)
    , m_operationRunning(false)
    , m_machineReady(false)
    , m_statusCheckTimer(nullptr)
{
	setupUI();
}

MachineStatusDialog::~MachineStatusDialog()
{
	if (m_statusCheckTimer)
	{
		m_statusCheckTimer->stop();
		delete m_statusCheckTimer;
	}
	qDeleteAll(m_statusItems);
	m_statusItems.clear();
}

void MachineStatusDialog::setupUI()
{
	m_mainLayout = new QVBoxLayout(this);
	m_mainLayout->setSpacing(10);

	m_progressLabel = new QLabel("检查设备状态...", this);
	m_progressLabel->setAlignment(Qt::AlignCenter);
	m_mainLayout->addWidget(m_progressLabel);

	StatusItem* item1 = new StatusItem;
	item1->name = "设备空闲状态";
	item1->label = new QLabel("● " + item1->name + ": 检测中...", this);
	item1->ok = false;
	item1->message = "";
	m_statusItems.append(item1);
	m_mainLayout->addWidget(item1->label);

	StatusItem* item2 = new StatusItem;
	item2->name = "设备模式";
	item2->label = new QLabel("● " + item2->name + ": 检测中...", this);
	item2->ok = false;
	item2->message = "";
	m_statusItems.append(item2);
	m_mainLayout->addWidget(item2->label);

	QFrame* line = new QFrame(this);
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	m_mainLayout->addWidget(line);

	m_progressBar = new QProgressBar(this);
	m_progressBar->setRange(0, 100);
	m_progressBar->setValue(0);
	m_progressBar->setVisible(false);
	m_mainLayout->addWidget(m_progressBar);
}

void MachineStatusDialog::init()
{
	setupAdditionalUI();

	m_statusCheckTimer = new QTimer(this);
	connect(m_statusCheckTimer, &QTimer::timeout, this, &MachineStatusDialog::checkStatus);
	m_statusCheckTimer->start(1000);

	checkStatus();
}

void MachineStatusDialog::closeEvent(QCloseEvent* event)
{
	if (m_operationRunning)
	{
		event->ignore();
	}
	else
	{
		if (m_statusCheckTimer)
		{
			m_statusCheckTimer->stop();
		}
		QDialog::closeEvent(event);
	}
}

void MachineStatusDialog::updateStatusDisplay()
{
	bool allOk = true;

	for (StatusItem* item : m_statusItems)
	{
		QString text;
		if (item->ok)
		{
			text = QString("<span style=\"color: green;\">● %1: ✓ 通过</span>").arg(item->name);
		}
		else
		{
			allOk = false;
			if (item->message.isEmpty())
			{
				text = QString("<span style=\"color: red;\">● %1: ✗ 未通过</span>").arg(item->name);
			}
			else
			{
				text = QString("<span style=\"color: red;\">● %1: ✗ %2</span>").arg(item->name).arg(item->message);
			}
		}
		item->label->setText(text);
	}

	if (allOk)
	{
		m_progressLabel->setText("<span style=\"color: green; font-weight: bold;\">✅ 所有检查项通过，设备就绪</span>");
	}
	else
	{
		m_progressLabel->setText("<span style=\"color: #cc6600; font-weight: bold;\">⚠️ 部分检查项未通过，请查看详情</span>");
	}

	m_machineReady = allOk;
}

void MachineStatusDialog::checkStatus()
{
	if (m_operationRunning)
		return;

	QString errorMessage;

	for (StatusItem* item : m_statusItems)
	{
		item->ok = false;
		item->message = "";
	}

	if (!m_pointCloudService)
	{
		if (m_statusItems.size() > 0)
		{
			m_statusItems[0]->ok = false;
			m_statusItems[0]->message = "PointCloudService未初始化";
		}
		updateStatusDisplay();
		updateUIState();
		return;
	}

	QString tempError;
	if (m_statusItems.size() > 0)
	{
		if (m_pointCloudService->waitForMachineIdle(1, &tempError))
		{
			m_statusItems[0]->ok = true;
		}
		else
		{
			m_statusItems[0]->ok = false;
			m_statusItems[0]->message = tempError.isEmpty() ? "设备未处于空闲状态" : tempError;
		}
	}

	if (m_statusItems.size() > 1)
	{
		QString mode;
		if (m_pointCloudService->getMachineMode(mode, &tempError))
		{
			if (mode == "Auto")
			{
				m_statusItems[1]->ok = true;
			}
			else
			{
				m_statusItems[1]->ok = false;
				m_statusItems[1]->message = QString("当前模式: %1 (需要: Auto)").arg(mode);
			}
		}
		else
		{
			m_statusItems[1]->ok = false;
			m_statusItems[1]->message = tempError.isEmpty() ? "获取设备模式失败" : tempError;
		}
	}

	m_machineReady = checkMachineStatus(errorMessage);

	updateStatusDisplay();
	updateUIState();
}

bool MachineStatusDialog::checkMachineStatus(QString& errorMessage)
{
	if (!m_pointCloudService)
	{
		errorMessage = "PointCloudService未初始化";
		return false;
	}

	QString tempError;
	if (!m_pointCloudService->waitForMachineIdle(1, &tempError))
	{
		if (tempError.isEmpty())
		{
			errorMessage = "设备未处于空闲状态";
		}
		else
		{
			errorMessage = tempError;
		}
		return false;
	}

	QString mode;
	if (!m_pointCloudService->getMachineMode(mode, &tempError))
	{
		if (tempError.isEmpty())
		{
			errorMessage = "获取设备模式失败";
		}
		else
		{
			errorMessage = tempError;
		}
		return false;
	}

	if (mode != "Auto")
	{
		errorMessage = QString("设备模式必须为Auto，当前模式: '%1'").arg(mode);
		return false;
	}

	return true;
}

void MachineStatusDialog::setOperationRunning(bool running)
{
	m_operationRunning = running;

	if (m_statusCheckTimer)
	{
		if (running)
		{
			m_statusCheckTimer->stop();
		}
		else
		{
			m_statusCheckTimer->start(1000);
		}
	}

	updateUIState();
}

void MachineStatusDialog::setProgressText(const QString& text)
{
	m_progressLabel->setText(text);
}

void MachineStatusDialog::setProgressValue(int value)
{
	m_progressBar->setValue(value);
}

void MachineStatusDialog::updateUIState()
{
	bool enabled = !m_operationRunning && m_machineReady;

	QList<QWidget*> widgets = findChildren<QWidget*>();
	for (QWidget* widget : widgets)
	{
		QPushButton* button = qobject_cast<QPushButton*>(widget);
		if (button)
		{
			if (button->text() == "取消")
			{
				button->setEnabled(!m_operationRunning);
			}
			else
			{
				button->setEnabled(enabled);
			}
		}
		else
		{
			widget->setEnabled(enabled);
		}
	}

	m_progressBar->setVisible(m_operationRunning);
}

void MachineStatusDialog::onStartOperation()
{
	if (!m_pointCloudService || m_operationRunning)
		return;

	MachineStatusDialogGuard guard(this);
	m_progressBar->setVisible(true);
	m_progressBar->setValue(0);

	onOperationStarted();

	bool success = performOperation();

	m_progressBar->setValue(100);

	onOperationCompleted(success);
}
