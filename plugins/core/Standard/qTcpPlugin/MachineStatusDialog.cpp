#include "MachineStatusDialog.h"
#include "PointCloudService.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
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
}

void MachineStatusDialog::setupUI()
{
	m_mainLayout = new QVBoxLayout(this);

	m_progressLabel = new QLabel("检查设备状态...", this);
	m_progressLabel->setAlignment(Qt::AlignCenter);
	m_mainLayout->addWidget(m_progressLabel);

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

void MachineStatusDialog::checkStatus()
{
	if (m_operationRunning)
		return;

	QString errorMessage;
	m_machineReady = checkMachineStatus(errorMessage);

	if (m_machineReady)
	{
		m_progressLabel->setText("✅ 设备就绪，可以开始操作");
	}
	else
	{
		m_progressLabel->setText(QString("⚠️ 设备未就绪: %1").arg(errorMessage));
	}

	updateUIState();
}

bool MachineStatusDialog::checkMachineStatus(QString& errorMessage)
{
	if (!m_pointCloudService)
	{
		errorMessage = "PointCloudService未初始化";
		return false;
	}

	if (!m_pointCloudService->waitForMachineIdle(1, &errorMessage))
	{
		if (errorMessage.isEmpty())
		{
			errorMessage = "设备未处于空闲状态";
		}
		return false;
	}

	QString mode;
	if (!m_pointCloudService->getMachineMode(mode, &errorMessage))
	{
		if (errorMessage.isEmpty())
		{
			errorMessage = "获取设备模式失败";
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
