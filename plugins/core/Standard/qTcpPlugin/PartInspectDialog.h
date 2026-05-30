#pragma once

#include "MachineStatusDialog.h"

class QComboBox;
class QLineEdit;
class QPushButton;
class QHBoxLayout;
class QLabel;
class QFrame;

class PartInspectDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit PartInspectDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~PartInspectDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QComboBox* m_partTypeCombo;
	QLineEdit* m_rfidEdit;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
	QLabel* m_schematicLabel;
	QLabel* m_instructionLabel;
	QFrame* m_infoFrame;
	
	void loadPartTypes();
};
