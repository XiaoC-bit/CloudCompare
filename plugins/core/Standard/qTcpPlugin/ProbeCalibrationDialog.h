#pragma once

#include "MachineStatusDialog.h"

class QPushButton;
class QHBoxLayout;
class QLabel;
class QFrame;

class ProbeCalibrationDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit ProbeCalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~ProbeCalibrationDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
	QLabel* m_schematicLabel;
	QLabel* m_instructionLabel;
	QFrame* m_infoFrame;
};
