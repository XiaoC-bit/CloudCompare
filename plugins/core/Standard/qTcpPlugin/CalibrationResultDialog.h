#pragma once

#include <QDialog>
#include <qframe.h>
#include <QJsonObject>
#include <QJsonArray>

class QLabel;
class QTableWidget;
class QPushButton;

class CalibrationResultDialog : public QDialog
{
	Q_OBJECT

public:
	explicit CalibrationResultDialog(QWidget* parent = nullptr);
	~CalibrationResultDialog() override;

private slots:
	void onCopyMatrix();

private:
	void initUI();
	void loadCalibrationResult();
	void loadProbeCalibrationResult();
	void updateResultOverview(const QJsonObject& result);
	void updateFitResultsTable(const QJsonArray& fitResults, const QJsonArray& residuals, double threshold);
	void updateMatrix(const QJsonArray& matrix);
	void updateStatistics(const QJsonArray& fitResults, const QJsonArray& residuals);

	QLabel* m_resultLabel;
	QLabel* m_positionCountLabel;
	QLabel* m_residualOkLabel;
	QLabel* m_residualThresholdLabel;
	QLabel* m_calibrationTimeLabel;
	QLabel* m_errorMessageLabel;
	QFrame* m_errorFrame;

	QTableWidget* m_fitResultsTable;
	QTableWidget* m_matrixTable;
	QPushButton* m_copyMatrixButton;
	QJsonArray m_currentMatrix;

	QLabel* m_maxResidualLabel;
	QLabel* m_minResidualLabel;
	QLabel* m_avgResidualLabel;
	QLabel* m_maxRmsLabel;
	QLabel* m_avgRmsLabel;

	QLabel* m_probeResultLabel;
	QLabel* m_probeOffsetLabel;
	QLabel* m_probeXLabel;
	QLabel* m_probeYLabel;
	QLabel* m_probeZLabel;
};
