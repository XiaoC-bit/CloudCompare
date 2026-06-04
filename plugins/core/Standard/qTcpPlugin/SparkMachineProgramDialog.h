#pragma once

#include "MachineStatusDialog.h"
#include <QMap>

class QComboBox;
class QLineEdit;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QFrame;

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
	void updateUIState() override;

private slots:
	void onPartTypeChanged(int index);

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
	
	QComboBox* m_partTypeCombo;
	QComboBox* m_electrodeTypeCombo;
	
	QMap<QString, QStringList> m_partElectrodeMap;
	
	QString m_lastError;
	
	void loadPartTypes();
	void loadElectrodesForPart(const QString& partName);
	void loadConfig();
	void saveConfigToFile();
};
