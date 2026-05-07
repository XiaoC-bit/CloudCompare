#pragma once

#include "MachineStatusDialog.h"

class QComboBox;
class QLineEdit;

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
};
