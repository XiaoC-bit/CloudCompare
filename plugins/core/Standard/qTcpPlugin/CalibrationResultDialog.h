#pragma once

#include <QDialog>
#include <QJsonObject>

class QLabel;
class QTableWidget;
class QTextEdit;

class CalibrationResultDialog : public QDialog
{
	Q_OBJECT

public:
	explicit CalibrationResultDialog(QWidget* parent = nullptr);
	~CalibrationResultDialog() override;

private:
	void initUI();
	void loadCalibrationResult();
	void updateResultOverview(const QJsonObject& result);
	void updateFitResultsTable(const QJsonArray& fitResults, const QJsonArray& residuals, double threshold);
	void updateMatrix(const QJsonArray& matrix);
	void updateStatistics(const QJsonArray& fitResults, const QJsonArray& residuals);

	QLabel* m_resultLabel;
	QLabel* m_statusLabel;
	QLabel* m_positionCountLabel;
	QLabel* m_residualOkLabel;
	QLabel* m_residualThresholdLabel;
	QTextEdit* m_messageTextEdit;

	QTableWidget* m_fitResultsTable;
	QTableWidget* m_matrixTable;

	QLabel* m_maxResidualLabel;
	QLabel* m_minResidualLabel;
	QLabel* m_avgResidualLabel;
	QLabel* m_maxRmsLabel;
	QLabel* m_avgRmsLabel;
};