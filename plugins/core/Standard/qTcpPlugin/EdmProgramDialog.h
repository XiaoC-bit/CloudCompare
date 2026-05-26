#pragma once

#include "MachineStatusDialog.h"
#include <Eigen/Dense>

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
	void onPasteMatrix();
	void onCalculateCompensation();

private:
	QTableWidget* m_matrixTable;
	QDoubleSpinBox* m_xSpinBox;
	QDoubleSpinBox* m_ySpinBox;
	QDoubleSpinBox* m_zSpinBox;
	QDoubleSpinBox* m_aSpinBox;
	QDoubleSpinBox* m_bSpinBox;
	QDoubleSpinBox* m_cSpinBox;

	bool parseMatrix(Eigen::Matrix4d& matrix);
	void calculateCompensation(const Eigen::Matrix4d& T_icp);
};
