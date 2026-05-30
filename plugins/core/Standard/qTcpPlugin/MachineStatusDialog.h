#pragma once

#include <QDialog>
#include <QProgressBar>
#include <QLabel>

class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class PointCloudService;
class ccMainAppInterface;
class QTimer;

class MachineStatusDialog : public QDialog
{
	Q_OBJECT

protected:
	friend class MachineStatusDialogGuard;

	MachineStatusDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	virtual ~MachineStatusDialog() override;

	virtual void setupUI();
	virtual void setupAdditionalUI() {};
	virtual void init();
	virtual void onOperationStarted() {}
	virtual bool performOperation() = 0;
	virtual void onOperationCompleted(bool success) {}

	void setOperationRunning(bool running);
	bool checkMachineStatus(QString& errorMessage);

	void setProgressText(const QString& text);
	void setProgressValue(int value);

	QVBoxLayout* m_mainLayout;
	QProgressBar* m_progressBar;
	QLabel* m_progressLabel;
	QTimer* m_statusCheckTimer;

	PointCloudService* m_pointCloudService;
	ccMainAppInterface* m_app;

	bool m_operationRunning;
	bool m_machineReady;

	struct StatusItem {
		QLabel* label;
		QString name;
		bool ok;
		QString message;
	};

	QList<StatusItem*> m_statusItems;

public slots:
	void onStartOperation();
	void checkStatus();

protected:
	void closeEvent(QCloseEvent* event) override;
	void updateStatusDisplay();

protected:
	virtual void updateUIState();
};

class MachineStatusDialogGuard
{
  public:
	MachineStatusDialog* dlg;
	explicit MachineStatusDialogGuard(MachineStatusDialog* d)
	    : dlg(d)
	{
		dlg->setOperationRunning(true);
	}
	~MachineStatusDialogGuard()
	{
		dlg->setOperationRunning(false);
	}
};
