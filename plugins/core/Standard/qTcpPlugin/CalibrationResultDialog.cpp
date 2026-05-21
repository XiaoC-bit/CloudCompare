#include "CalibrationResultDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextEdit>
#include <QHeaderView>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonParseError>
#include <QCoreApplication>
#include <QFrame>

CalibrationResultDialog::CalibrationResultDialog(QWidget* parent)
    : QDialog(parent)
{
	setWindowTitle("相机手眼标定结果");
	setMinimumWidth(900);
	setMinimumHeight(700);
	initUI();
	loadCalibrationResult();
}

CalibrationResultDialog::~CalibrationResultDialog()
{
}

void CalibrationResultDialog::initUI()
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setContentsMargins(16, 16, 16, 16);
	mainLayout->setSpacing(16);

	QFrame* overviewFrame = new QFrame();
	overviewFrame->setFrameShape(QFrame::Box);
	overviewFrame->setStyleSheet("QFrame { border: 1px solid #d0d0d0; border-radius: 4px; }");
	QVBoxLayout* overviewLayout = new QVBoxLayout(overviewFrame);
	overviewLayout->setContentsMargins(12, 12, 12, 12);
	overviewLayout->setSpacing(8);

	QLabel* overviewTitle = new QLabel("结果概览");
	QFont titleFont = overviewTitle->font();
	titleFont.setBold(true);
	titleFont.setPointSize(titleFont.pointSize() + 1);
	overviewTitle->setFont(titleFont);
	overviewLayout->addWidget(overviewTitle);

	QGridLayout* gridLayout = new QGridLayout();
	gridLayout->setSpacing(12);

	gridLayout->addWidget(new QLabel("标定结果:"), 0, 0);
	m_resultLabel = new QLabel("--");
	gridLayout->addWidget(m_resultLabel, 0, 1);

	gridLayout->addWidget(new QLabel("任务状态:"), 0, 2);
	m_statusLabel = new QLabel("--");
	gridLayout->addWidget(m_statusLabel, 0, 3);

	gridLayout->addWidget(new QLabel("有效点数:"), 1, 0);
	m_positionCountLabel = new QLabel("--");
	gridLayout->addWidget(m_positionCountLabel, 1, 1);

	gridLayout->addWidget(new QLabel("残差是否合格:"), 1, 2);
	m_residualOkLabel = new QLabel("--");
	gridLayout->addWidget(m_residualOkLabel, 1, 3);

	gridLayout->addWidget(new QLabel("残差阈值:"), 2, 0);
	m_residualThresholdLabel = new QLabel("--");
	gridLayout->addWidget(m_residualThresholdLabel, 2, 1);

	overviewLayout->addLayout(gridLayout);

	QLabel* messageLabel = new QLabel("过程消息:");
	overviewLayout->addWidget(messageLabel);

	m_messageTextEdit = new QTextEdit();
	m_messageTextEdit->setReadOnly(true);
	m_messageTextEdit->setMaximumHeight(60);
	m_messageTextEdit->setStyleSheet("QTextEdit { border: 1px solid #d0d0d0; border-radius: 3px; }");
	overviewLayout->addWidget(m_messageTextEdit);

	mainLayout->addWidget(overviewFrame);

	QLabel* fitResultsTitle = new QLabel("采样点明细");
	fitResultsTitle->setFont(titleFont);
	mainLayout->addWidget(fitResultsTitle);

	m_fitResultsTable = new QTableWidget();
	m_fitResultsTable->setColumnCount(9);
	m_fitResultsTable->setHorizontalHeaderLabels({
		"点号", "Machine X", "Machine Y", "Machine Z",
		"Scanner X", "Scanner Y", "Scanner Z", "RMS", "Residual"
	});
	m_fitResultsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_fitResultsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
	m_fitResultsTable->setAlternatingRowColors(true);
	mainLayout->addWidget(m_fitResultsTable, 1);

	QLabel* matrixTitle = new QLabel("Scanner → Machine 变换矩阵");
	matrixTitle->setFont(titleFont);
	mainLayout->addWidget(matrixTitle);

	m_matrixTable = new QTableWidget();
	m_matrixTable->setColumnCount(4);
	m_matrixTable->setRowCount(4);
	m_matrixTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
	m_matrixTable->horizontalHeader()->setVisible(false);
	m_matrixTable->verticalHeader()->setVisible(false);
	m_matrixTable->setMaximumHeight(180);
	m_matrixTable->setStyleSheet(R"(
		QTableWidget { border: 1px solid #d0d0d0; border-radius: 4px; }
		QTableWidget::item { 
			text-align: center; 
			font-family: monospace;
			padding: 4px;
		}
	)");
	mainLayout->addWidget(m_matrixTable);

	QFrame* statsFrame = new QFrame();
	statsFrame->setFrameShape(QFrame::Box);
	statsFrame->setStyleSheet("QFrame { border: 1px solid #d0d0d0; border-radius: 4px; }");
	QVBoxLayout* statsLayout = new QVBoxLayout(statsFrame);
	statsLayout->setContentsMargins(12, 12, 12, 12);

	QLabel* statsTitle = new QLabel("统计信息");
	statsTitle->setFont(titleFont);
	statsLayout->addWidget(statsTitle);

	QGridLayout* statsGrid = new QGridLayout();
	statsGrid->setSpacing(12);

	statsGrid->addWidget(new QLabel("最大 Residual:"), 0, 0);
	m_maxResidualLabel = new QLabel("--");
	statsGrid->addWidget(m_maxResidualLabel, 0, 1);

	statsGrid->addWidget(new QLabel("最小 Residual:"), 0, 2);
	m_minResidualLabel = new QLabel("--");
	statsGrid->addWidget(m_minResidualLabel, 0, 3);

	statsGrid->addWidget(new QLabel("平均 Residual:"), 1, 0);
	m_avgResidualLabel = new QLabel("--");
	statsGrid->addWidget(m_avgResidualLabel, 1, 1);

	statsGrid->addWidget(new QLabel("最大 RMS:"), 1, 2);
	m_maxRmsLabel = new QLabel("--");
	statsGrid->addWidget(m_maxRmsLabel, 1, 3);

	statsGrid->addWidget(new QLabel("平均 RMS:"), 2, 0);
	m_avgRmsLabel = new QLabel("--");
	statsGrid->addWidget(m_avgRmsLabel, 2, 1);

	statsLayout->addLayout(statsGrid);
	mainLayout->addWidget(statsFrame);
}

void CalibrationResultDialog::loadCalibrationResult()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QString filePath = appDir + "/Template/camera_calibration_status.json";

	QFile file(filePath);
	if (!file.exists()) {
		m_messageTextEdit->setText("标定结果文件不存在: " + filePath);
		return;
	}

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		m_messageTextEdit->setText("无法打开标定结果文件");
		return;
	}

	QByteArray data = file.readAll();
	file.close();

	QJsonParseError parseError;
	QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
	if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
		m_messageTextEdit->setText("无效的JSON格式: " + parseError.errorString());
		return;
	}

	QJsonObject root = doc.object();

	if (root.contains("Message")) {
		m_messageTextEdit->setText(root["Message"].toString());
	}

	if (root.contains("Result")) {
		QString result = root["Result"].toString();
		m_resultLabel->setText(result);
		m_resultLabel->setStyleSheet(result == "OK" ? "color: #2ecc71;" : "color: #e74c3c;");
	}

	if (root.contains("Status")) {
		m_statusLabel->setText(root["Status"].toString());
	}

	if (root.contains("CalibrationResult")) {
		QJsonObject calibrationResult = root["CalibrationResult"].toObject();
		updateResultOverview(calibrationResult);

		if (calibrationResult.contains("FitResults")) {
			QJsonArray fitResults = calibrationResult["FitResults"].toArray();
			QJsonArray residuals = calibrationResult.contains("Residuals") 
				? calibrationResult["Residuals"].toArray() 
				: QJsonArray();
			double threshold = calibrationResult.contains("ResidualThreshold") 
				? calibrationResult["ResidualThreshold"].toDouble() 
				: 0.12;
			updateFitResultsTable(fitResults, residuals, threshold);
		}

		if (calibrationResult.contains("Matrix")) {
			updateMatrix(calibrationResult["Matrix"].toArray());
		}

		if (calibrationResult.contains("FitResults") && calibrationResult.contains("Residuals")) {
			updateStatistics(calibrationResult["FitResults"].toArray(), 
							calibrationResult["Residuals"].toArray());
		}
	}
}

void CalibrationResultDialog::updateResultOverview(const QJsonObject& result)
{
	if (result.contains("Result")) {
		QString res = result["Result"].toString();
		m_resultLabel->setText(res);
		m_resultLabel->setStyleSheet(res == "OK" ? "color: #2ecc71;" : "color: #e74c3c;");
	}

	if (result.contains("PositionCount")) {
		m_positionCountLabel->setText(QString::number(result["PositionCount"].toInt()));
	}

	if (result.contains("ResidualOk")) {
		bool ok = result["ResidualOk"].toBool();
		m_residualOkLabel->setText(ok ? "合格" : "不合格");
		m_residualOkLabel->setStyleSheet(ok ? "color: #2ecc71;" : "color: #e74c3c;");
	}

	if (result.contains("ResidualThreshold")) {
		m_residualThresholdLabel->setText(QString::number(result["ResidualThreshold"].toDouble(), 'f', 4));
	}
}

void CalibrationResultDialog::updateFitResultsTable(const QJsonArray& fitResults, 
													const QJsonArray& residuals, 
													double threshold)
{
	m_fitResultsTable->setRowCount(fitResults.size());

	for (int i = 0; i < fitResults.size(); ++i) {
		QJsonObject fitResult = fitResults[i].toObject();

		QTableWidgetItem* indexItem = new QTableWidgetItem(QString::number(fitResult["index"].toInt()));
		m_fitResultsTable->setItem(i, 0, indexItem);

		QJsonArray machine = fitResult["machine"].toArray();
		for (int j = 0; j < 3 && j < machine.size(); ++j) {
			QTableWidgetItem* item = new QTableWidgetItem(QString::number(machine[j].toDouble(), 'f', 6));
			m_fitResultsTable->setItem(i, j + 1, item);
		}

		QJsonArray scanner = fitResult["scanner"].toArray();
		for (int j = 0; j < 3 && j < scanner.size(); ++j) {
			QTableWidgetItem* item = new QTableWidgetItem(QString::number(scanner[j].toDouble(), 'f', 6));
			m_fitResultsTable->setItem(i, j + 4, item);
		}

		double rms = fitResult["rms"].toDouble();
		QTableWidgetItem* rmsItem = new QTableWidgetItem(QString::number(rms, 'f', 6));
		if (rms > threshold) {
			rmsItem->setBackground(QColor(255, 220, 220));
		}
		m_fitResultsTable->setItem(i, 7, rmsItem);

		double residual = (i < residuals.size()) ? residuals[i].toDouble() : 0;
		QTableWidgetItem* residualItem = new QTableWidgetItem(QString::number(residual, 'f', 6));
		if (residual > threshold) {
			residualItem->setBackground(QColor(255, 220, 220));
		}
		m_fitResultsTable->setItem(i, 8, residualItem);
	}
}

void CalibrationResultDialog::updateMatrix(const QJsonArray& matrix)
{
	for (int i = 0; i < 4 && i < matrix.size(); ++i) {
		QJsonArray row = matrix[i].toArray();
		for (int j = 0; j < 4 && j < row.size(); ++j) {
			QTableWidgetItem* item = new QTableWidgetItem(QString::number(row[j].toDouble(), 'f', 6));
			m_matrixTable->setItem(i, j, item);
		}
	}
	m_matrixTable->resizeColumnsToContents();
}

void CalibrationResultDialog::updateStatistics(const QJsonArray& fitResults, const QJsonArray& residuals)
{
	if (residuals.isEmpty()) {
		return;
	}

	double maxResidual = residuals[0].toDouble();
	double minResidual = residuals[0].toDouble();
	double sumResidual = 0;

	for (int i = 0; i < residuals.size(); ++i) {
		double val = residuals[i].toDouble();
		maxResidual = qMax(maxResidual, val);
		minResidual = qMin(minResidual, val);
		sumResidual += val;
	}

	m_maxResidualLabel->setText(QString::number(maxResidual, 'f', 6));
	m_minResidualLabel->setText(QString::number(minResidual, 'f', 6));
	m_avgResidualLabel->setText(QString::number(sumResidual / residuals.size(), 'f', 6));

	double maxRms = 0;
	double sumRms = 0;
	int rmsCount = 0;

	for (int i = 0; i < fitResults.size(); ++i) {
		QJsonObject fitResult = fitResults[i].toObject();
		if (fitResult.contains("rms")) {
			double rms = fitResult["rms"].toDouble();
			maxRms = qMax(maxRms, rms);
			sumRms += rms;
			rmsCount++;
		}
	}

	if (rmsCount > 0) {
		m_maxRmsLabel->setText(QString::number(maxRms, 'f', 6));
		m_avgRmsLabel->setText(QString::number(sumRms / rmsCount, 'f', 6));
	}
}