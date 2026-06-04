#pragma once

#include "MachineStatusDialog.h"

class QComboBox;
class QLineEdit;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;

class SparkMachineProgramDialog : public MachineStatusDialog
{
	Q_OBJECT

public:
	explicit SparkMachineProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~SparkMachineProgramDialog() override;

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	QComboBox* m_machineTypeCombo;
	QLineEdit* m_partRfidEdit;
	QLineEdit* m_electrodeRfidEdit;
	
	// U轴中心
	QDoubleSpinBox* m_uAxisCenterXSpin;
	QDoubleSpinBox* m_uAxisCenterYSpin;
	QDoubleSpinBox* m_uAxisCenterZSpin;
	
	// V轴中心
	QDoubleSpinBox* m_vAxisCenterXSpin;
	QDoubleSpinBox* m_vAxisCenterYSpin;
	QDoubleSpinBox* m_vAxisCenterZSpin;
	
	// W轴中心
	QDoubleSpinBox* m_wAxisCenterXSpin;
	QDoubleSpinBox* m_wAxisCenterYSpin;
	QDoubleSpinBox* m_wAxisCenterZSpin;
	
	// 上夹具中心
	QDoubleSpinBox* m_upChuckCenterXSpin;
	QDoubleSpinBox* m_upChuckCenterYSpin;
	QDoubleSpinBox* m_upChuckCenterZSpin;
	
	// 下夹具中心
	QDoubleSpinBox* m_downChuckCenterXSpin;
	QDoubleSpinBox* m_downChuckCenterYSpin;
	QDoubleSpinBox* m_downChuckCenterZSpin;
	
	QLineEdit* m_partTypeEdit;
	QLineEdit* m_electrodeTypeEdit;
	
	QString m_lastError;
	
	void loadConfig();
	void saveConfigToFile();
};
