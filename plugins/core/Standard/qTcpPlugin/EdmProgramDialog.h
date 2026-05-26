#pragma once

#include "MachineStatusDialog.h"
#include <Eigen/Dense>

class QLineEdit;
class QLabel;
class QDoubleSpinBox;
class QTableWidget;

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

private slots:
	void onCalculateCompensation();

private:
	QTableWidget* m_matrixTable;
	QLabel* m_calculatedXLabel;
	QLabel* m_calculatedYLabel;
	QLabel* m_calculatedZLabel;
	QLabel* m_calculatedBLabel;
	QLabel* m_calculatedCLabel;

	double m_calculatedX = 0;
	double m_calculatedY = 0;
	double m_calculatedZ = 0;
	double m_calculatedB = 0;
	double m_calculatedC = 0;

	bool parseMatrix(Eigen::Matrix4d& matrix);
	void calculateCompensation(const Eigen::Matrix4d& T_icp);
};
