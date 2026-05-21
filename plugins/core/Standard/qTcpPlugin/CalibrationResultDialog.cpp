#include "CalibrationResultDialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <qfileinfo.h>
#include <qdatetime.h>
#include <QHeaderView>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonParseError>
#include <QCoreApplication>
#include <QFrame>
#include <QPushButton>
#include <QClipboard>
#include <QApplication>

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
    mainLayout->setContentsMargins(20, 16, 20, 16);
    mainLayout->setSpacing(10);

    // ── 1. 结果概览 ──────────────────────────────────────────────────────────
    {
        QFrame* card = new QFrame();
        card->setFrameShape(QFrame::NoFrame);
        card->setStyleSheet(R"(
            QFrame {
                background-color: #ffffff;
                border: 1px solid #d0d0d0;
                border-radius: 4px;
            }
        )");

        QVBoxLayout* cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(14, 12, 14, 12);
        cardLayout->setSpacing(10);

        QLabel* titleLabel = new QLabel("结果概览");
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleFont.setPointSize(titleFont.pointSize() + 1);
        titleLabel->setFont(titleFont);
        titleLabel->setStyleSheet("border: none; background: transparent;");
        cardLayout->addWidget(titleLabel);

        QFrame* sep = new QFrame();
        sep->setFrameShape(QFrame::HLine);
        sep->setStyleSheet("border: none; background-color: #e0e0e0; max-height: 1px;");
        cardLayout->addWidget(sep);

        QGridLayout* grid = new QGridLayout();
        grid->setHorizontalSpacing(12);
        grid->setVerticalSpacing(8);
        grid->setColumnMinimumWidth(1, 110);
        grid->setColumnMinimumWidth(3, 110);

        auto addRow = [&](int row, int col, const QString& key, QLabel*& valueLabel) {
            QLabel* keyLbl = new QLabel(key);
            keyLbl->setStyleSheet("color: #444444; background: transparent; border: none;");
            grid->addWidget(keyLbl, row, col * 2);
            valueLabel = new QLabel("--");
            valueLabel->setStyleSheet("color: #111111; font-weight: 600; background: transparent; border: none;");
            grid->addWidget(valueLabel, row, col * 2 + 1);
        };

        addRow(0, 0, "标定结果:",     m_resultLabel);
        addRow(0, 1, "有效点数:",     m_positionCountLabel);
        addRow(1, 0, "残差是否合格:", m_residualOkLabel);
        addRow(1, 1, "残差阈值:",     m_residualThresholdLabel);
        addRow(2, 0, "标定时间:",     m_calibrationTimeLabel);

        cardLayout->addLayout(grid);

        mainLayout->addWidget(card);
    }

    // ── 2. 采样点明细 ─────────────────────────────────────────────────────────
    {
        QLabel* sectionLabel = new QLabel("采样点明细");
        QFont f = sectionLabel->font();
        f.setBold(true);
        f.setPointSize(f.pointSize() + 1);
        sectionLabel->setFont(f);
        mainLayout->addWidget(sectionLabel);

        m_fitResultsTable = new QTableWidget();
        m_fitResultsTable->setColumnCount(9);
        m_fitResultsTable->setHorizontalHeaderLabels({
            "#", "Machine X", "Machine Y", "Machine Z",
            "Scanner X", "Scanner Y", "Scanner Z", "RMS", "Residual"
        });
        m_fitResultsTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_fitResultsTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        m_fitResultsTable->setColumnWidth(0, 40);
        m_fitResultsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_fitResultsTable->setAlternatingRowColors(true);
        m_fitResultsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_fitResultsTable->setShowGrid(false);
        m_fitResultsTable->verticalHeader()->setVisible(false);
        m_fitResultsTable->horizontalHeader()->setHighlightSections(false);
        m_fitResultsTable->verticalHeader()->setDefaultSectionSize(36);
        m_fitResultsTable->setStyleSheet(R"(
            QTableWidget {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                background-color: #ffffff;
                alternate-background-color: #f7f7f9;
                outline: none;
            }
            QHeaderView::section {
                background-color: #f0f0f2;
                color: #444444;
                font-weight: bold;
                padding: 4px 8px;
                border: none;
                border-bottom: 1px solid #d0d0d0;
            }
            QTableWidget::item {
                padding: 0px 4px;
            }
            QTableWidget::item:selected {
                background-color: #dce8ff;
                color: #000000;
            }
        )");

        mainLayout->addWidget(m_fitResultsTable, 1);
    }

    // ── 3. 变换矩阵 ───────────────────────────────────────────────────────────
    {
        QHBoxLayout* headerLayout = new QHBoxLayout();

        QLabel* sectionLabel = new QLabel("Scanner → Machine 变换矩阵");
        QFont f = sectionLabel->font();
        f.setBold(true);
        f.setPointSize(f.pointSize() + 1);
        sectionLabel->setFont(f);
        headerLayout->addWidget(sectionLabel);

        headerLayout->addStretch();

        m_copyMatrixButton = new QPushButton("复制矩阵");
        m_copyMatrixButton->setFixedHeight(28);
        m_copyMatrixButton->setFixedWidth(80);
        m_copyMatrixButton->setStyleSheet(R"(
            QPushButton {
                color: #3366cc;
                border: 1px solid #3366cc;
                border-radius: 3px;
                background: transparent;
                font-size: 12px;
                padding: 0 6px;
            }
            QPushButton:hover {
                background-color: #f0f5ff;
            }
        )");
        connect(m_copyMatrixButton, &QPushButton::clicked, this, &CalibrationResultDialog::onCopyMatrix);
        headerLayout->addWidget(m_copyMatrixButton);

        mainLayout->addLayout(headerLayout);

        m_matrixTable = new QTableWidget();
        m_matrixTable->setColumnCount(4);
        m_matrixTable->setRowCount(4);
        m_matrixTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_matrixTable->horizontalHeader()->setVisible(false);
        m_matrixTable->verticalHeader()->setVisible(false);
        m_matrixTable->setFixedHeight(148);
        m_matrixTable->setAlternatingRowColors(true);
        m_matrixTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_matrixTable->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        m_matrixTable->setStyleSheet(R"(
            QTableWidget {
                border: 1px solid #d0d0d0;
                border-radius: 4px;
                background-color: #ffffff;
                alternate-background-color: #f7f7f9;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 12px;
                outline: none;
            }
            QTableWidget::item {
                padding: 4px 10px;
            }
            QTableWidget::item:selected {
                background-color: #dce8ff;
                color: #000000;
            }
        )");

        mainLayout->addWidget(m_matrixTable);
    }

    // ── 4. 统计信息 ───────────────────────────────────────────────────────────
    {
        QFrame* card = new QFrame();
        card->setFrameShape(QFrame::NoFrame);
        card->setStyleSheet(R"(
            QFrame {
                background-color: #ffffff;
                border: 1px solid #d0d0d0;
                border-radius: 4px;
            }
        )");

        QVBoxLayout* cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(14, 12, 14, 12);
        cardLayout->setSpacing(10);

        QLabel* titleLabel = new QLabel("统计信息");
        QFont f = titleLabel->font();
        f.setBold(true);
        f.setPointSize(f.pointSize() + 1);
        titleLabel->setFont(f);
        titleLabel->setStyleSheet("border: none; background: transparent;");
        cardLayout->addWidget(titleLabel);

        QFrame* sep = new QFrame();
        sep->setFrameShape(QFrame::HLine);
        sep->setStyleSheet("border: none; background-color: #e0e0e0; max-height: 1px;");
        cardLayout->addWidget(sep);

        QGridLayout* grid = new QGridLayout();
        grid->setHorizontalSpacing(12);
        grid->setVerticalSpacing(8);
        grid->setColumnMinimumWidth(1, 110);
        grid->setColumnMinimumWidth(3, 110);

        auto addStat = [&](int row, int col, const QString& key, QLabel*& lbl) {
            QLabel* k = new QLabel(key);
            k->setStyleSheet("color: #444444; background: transparent; border: none;");
            grid->addWidget(k, row, col * 2);
            lbl = new QLabel("--");
            lbl->setStyleSheet("color: #111111; font-weight: 600; background: transparent; border: none;");
            grid->addWidget(lbl, row, col * 2 + 1);
        };

        addStat(0, 0, "最大 Residual:", m_maxResidualLabel);
        addStat(0, 1, "最小 Residual:", m_minResidualLabel);
        addStat(1, 0, "平均 Residual:", m_avgResidualLabel);
        addStat(1, 1, "最大 RMS:",      m_maxRmsLabel);
        addStat(2, 0, "平均 RMS:",      m_avgRmsLabel);

        cardLayout->addLayout(grid);
        mainLayout->addWidget(card);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  逻辑部分与原版完全相同，仅调整高亮色以匹配浅色主题
// ─────────────────────────────────────────────────────────────────────────────

void CalibrationResultDialog::loadCalibrationResult()
{
    QString appDir = QCoreApplication::applicationDirPath();
    QString filePath = appDir + "/Template/camera_calibration_status.json";

    QFile file(filePath);
    if (!file.exists()) {
        return;
    }

    QFileInfo fileInfo(file);
    QString modificationTime = fileInfo.lastModified().toString("yyyy-MM-dd HH:mm:ss");
    m_calibrationTimeLabel->setText(modificationTime);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        return;
    }

    QJsonObject root = doc.object();

    if (root.contains("Result")) {
        QString result = root["Result"].toString();
        m_resultLabel->setText(result);
        m_resultLabel->setStyleSheet(result == "OK"
            ? "color: #2e8b57; font-weight: 700; background: transparent; border: none;"
            : "color: #cc3333; font-weight: 700; background: transparent; border: none;");
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
            m_currentMatrix = calibrationResult["Matrix"].toArray();
            updateMatrix(m_currentMatrix);
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
        m_resultLabel->setStyleSheet(res == "OK"
            ? "color: #2e8b57; font-weight: 700; background: transparent; border: none;"
            : "color: #cc3333; font-weight: 700; background: transparent; border: none;");
    }

    if (result.contains("PositionCount")) {
        m_positionCountLabel->setText(QString::number(result["PositionCount"].toInt()));
    }

    if (result.contains("ResidualOk")) {
        bool ok = result["ResidualOk"].toBool();
        m_residualOkLabel->setText(ok ? "合格" : "不合格");
        m_residualOkLabel->setStyleSheet(ok
            ? "color: #2e8b57; font-weight: 700; background: transparent; border: none;"
            : "color: #cc3333; font-weight: 700; background: transparent; border: none;");
    }

    if (result.contains("ResidualThreshold")) {
        m_residualThresholdLabel->setText(
            QString::number(result["ResidualThreshold"].toDouble(), 'f', 4));
    }
}

void CalibrationResultDialog::updateFitResultsTable(const QJsonArray& fitResults,
                                                     const QJsonArray& residuals,
                                                     double threshold)
{
    m_fitResultsTable->setRowCount(fitResults.size());

    for (int i = 0; i < fitResults.size(); ++i) {
        QJsonObject fitResult = fitResults[i].toObject();

        auto setItem = [&](int col, const QString& text, bool warn = false) {
            QTableWidgetItem* item = new QTableWidgetItem(text);
            item->setTextAlignment(Qt::AlignCenter);
            if (warn) {
                item->setBackground(QColor(255, 220, 220));
                item->setForeground(QColor(180, 0, 0));
            }
            m_fitResultsTable->setItem(i, col, item);
        };

        setItem(0, QString::number(fitResult["index"].toInt()));

        QJsonArray machine = fitResult["machine"].toArray();
        for (int j = 0; j < 3 && j < machine.size(); ++j)
            setItem(j + 1, QString::number(machine[j].toDouble(), 'f', 6));

        QJsonArray scanner = fitResult["scanner"].toArray();
        for (int j = 0; j < 3 && j < scanner.size(); ++j)
            setItem(j + 4, QString::number(scanner[j].toDouble(), 'f', 6));

        double rms = fitResult["rms"].toDouble();
        setItem(7, QString::number(rms, 'f', 6), rms > threshold);

        double residual = (i < residuals.size()) ? residuals[i].toDouble() : 0.0;
        setItem(8, QString::number(residual, 'f', 6), residual > threshold);
    }
}

void CalibrationResultDialog::updateMatrix(const QJsonArray& matrix)
{
    for (int i = 0; i < 4 && i < matrix.size(); ++i) {
        QJsonArray row = matrix[i].toArray();
        for (int j = 0; j < 4 && j < row.size(); ++j) {
            QTableWidgetItem* item = new QTableWidgetItem(
                QString::number(row[j].toDouble(), 'f', 6));
            item->setTextAlignment(Qt::AlignCenter);
            // 最后一列（平移）用蓝色区分旋转部分
            item->setForeground(j == 3 ? QColor(0x33, 0x66, 0xcc) : QColor(0x22, 0x22, 0x22));
            m_matrixTable->setItem(i, j, item);
        }
    }
    m_matrixTable->resizeColumnsToContents();
}

void CalibrationResultDialog::updateStatistics(const QJsonArray& fitResults,
                                               const QJsonArray& residuals)
{
    if (residuals.isEmpty()) return;

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

    double maxRms = 0, sumRms = 0;
    int rmsCount = 0;

    for (int i = 0; i < fitResults.size(); ++i) {
        QJsonObject fitResult = fitResults[i].toObject();
        if (fitResult.contains("rms")) {
            double rms = fitResult["rms"].toDouble();
            maxRms  = qMax(maxRms, rms);
            sumRms += rms;
            rmsCount++;
        }
    }

    if (rmsCount > 0) {
        m_maxRmsLabel->setText(QString::number(maxRms, 'f', 6));
        m_avgRmsLabel->setText(QString::number(sumRms / rmsCount, 'f', 6));
    }
}

void CalibrationResultDialog::onCopyMatrix()
{
    QString matrixText;

    for (int i = 0; i < 4 && i < m_currentMatrix.size(); ++i) {
        QJsonArray row = m_currentMatrix[i].toArray();
        for (int j = 0; j < 4 && j < row.size(); ++j) {
            if (j > 0) {
                matrixText += " ";
            }
            matrixText += QString("%1").arg(row[j].toDouble(), 0, 'f', 8);
        }
        if (i < 3) {
            matrixText += "\n";
        }
    }

    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(matrixText);
}
