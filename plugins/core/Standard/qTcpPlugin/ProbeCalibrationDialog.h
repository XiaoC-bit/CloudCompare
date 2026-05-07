#pragma once

#include <QDialog>
#include <QProgressBar>
#include <QLabel>

class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class PointCloudService;
class ccMainAppInterface;

class ProbeCalibrationDialog : public QDialog
{
private:
	friend class ProbeCalibrationGuard;

public:
	explicit ProbeCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~ProbeCalibrationDialog() override;

private slots:
	void onStartCalibration();

private:
	void setupUI();
	void setCalibrationRunning(bool running);

	QVBoxLayout* m_mainLayout;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
	QProgressBar* m_progressBar;
	QLabel* m_progressLabel;

	PointCloudService* m_pointCloudService;
	ccMainAppInterface* m_app;

	bool m_calibrationRunning = false;

  protected:
	void closeEvent(QCloseEvent* event) override;
};

class ProbeCalibrationGuard
{
  public:
	ProbeCalibrationDialog* dlg;
	explicit ProbeCalibrationGuard(ProbeCalibrationDialog* d)
	    : dlg(d)
	{
		dlg->setCalibrationRunning(true);
	}
	~ProbeCalibrationGuard()
	{
		dlg->setCalibrationRunning(false);
	}
};
