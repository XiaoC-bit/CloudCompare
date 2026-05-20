#pragma once

#include <QDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>

class ccMainAppInterface;

class AcquirePcdDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AcquirePcdDialog(ccMainAppInterface* app, QWidget* parent = nullptr);
    ~AcquirePcdDialog() override;

private slots:
    void onAcquireClicked();
    void onCancelClicked();
    void onAcquisitionProgress();
    void onAcquisitionFinished(bool success, const QString& errorMessage);

private:
    void setupUI();
    bool acquirePointCloud(const QString& outputName);

    ccMainAppInterface* m_app;
    QProgressBar* m_progressBar;
    QLabel* m_statusLabel;
    QPushButton* m_acquireBtn;
    QPushButton* m_cancelBtn;
    QTimer* m_progressTimer;
    bool m_isAcquiring;
    bool m_cancelRequested;
};