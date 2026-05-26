#pragma once

#include "MachineStatusDialog.h"

class QLineEdit;
class QLabel;
class QDoubleSpinBox;

class EdmProgramDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit EdmProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~EdmProgramDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QDoubleSpinBox* m_xSpinBox;
	QDoubleSpinBox* m_ySpinBox;
	QDoubleSpinBox* m_zSpinBox;
	QDoubleSpinBox* m_bSpinBox;
	QDoubleSpinBox* m_cSpinBox;
};
