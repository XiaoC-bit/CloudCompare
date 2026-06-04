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
	
	// UVW旋转中心
	QDoubleSpinBox* m_uvwCenterXSpin;
	QDoubleSpinBox* m_uvwCenterYSpin;
	QDoubleSpinBox* m_uvwCenterZSpin;
	
	// 上卡盘中心
	QDoubleSpinBox* m_topChuckCenterXSpin;
	QDoubleSpinBox* m_topChuckCenterYSpin;
	QDoubleSpinBox* m_topChuckCenterZSpin;
	
	// 下卡盘中心
	QDoubleSpinBox* m_bottomChuckCenterXSpin;
	QDoubleSpinBox* m_bottomChuckCenterYSpin;
	QDoubleSpinBox* m_bottomChuckCenterZSpin;
	
	QLineEdit* m_partTypeEdit;
	QLineEdit* m_electrodeTypeEdit;
	
	QString m_lastError;
};
