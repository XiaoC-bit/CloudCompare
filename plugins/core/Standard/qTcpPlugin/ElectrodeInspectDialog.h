#pragma once

#include "MachineStatusDialog.h"

class QComboBox;
class QLineEdit;
class QPushButton;
class QHBoxLayout;

class ElectrodeInspectDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit ElectrodeInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~ElectrodeInspectDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QComboBox* m_electrodeTypeCombo;
	QLineEdit* m_rfidEdit;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
};
