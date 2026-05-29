#pragma once

#include "MachineStatusDialog.h"

class QPushButton;
class QHBoxLayout;

class ccMainAppInterface;
class PointCloudService;

class RingCalibrationDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit RingCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~RingCalibrationDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
};
