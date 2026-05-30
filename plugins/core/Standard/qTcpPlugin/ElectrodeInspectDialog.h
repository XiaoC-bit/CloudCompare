#pragma once

#include<qmap.h>
#include "MachineStatusDialog.h"

class QComboBox;
class QLineEdit;
class QPushButton;
class QHBoxLayout;
class QLabel;
class QFrame;

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
	void updateUIState() override;

private slots:
	void onPartTypeChanged(int index);

private:
	QComboBox* m_partTypeCombo;
	QComboBox* m_electrodeTypeCombo;
	QLineEdit* m_rfidEdit;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
	QLabel* m_schematicLabel;
	QLabel* m_instructionLabel;
	QFrame* m_infoFrame;
	
	QMap<QString, QStringList> m_partElectrodeMap;
	
	void loadPartTypes();
	void loadElectrodesForPart(const QString& partName);
};
