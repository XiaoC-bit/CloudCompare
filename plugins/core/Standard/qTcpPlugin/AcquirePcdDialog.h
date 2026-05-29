#pragma once

#include <QDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QRadioButton>
#include <QLineEdit>
#include <QGroupBox>

class ccMainAppInterface;
class PointCloudService;

class AcquirePcdDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AcquirePcdDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
    ~AcquirePcdDialog() override;

private slots:
    void onAcquireClicked();
    void onCancelClicked();
    void onAcquisitionProgress();
    void onAcquisitionFinished(bool success, const QString& errorMessage);
    void onRadioButtonChanged();
    void onGetCapturePositionFromDevice();
    void onGetModelOriginFromDevice();
    void onSaveConfig();

private:
    void setupUI();
    bool acquirePointCloud(const QString& outputName);
    void loadConfig();
    void saveConfigToFile();

    QPushButton* m_saveConfigBtn;

    ccMainAppInterface* m_app;
    PointCloudService* m_pointCloudService;
    QProgressBar* m_progressBar;
    QLabel* m_statusLabel;
    QPushButton* m_acquireBtn;
    QPushButton* m_cancelBtn;
    QTimer* m_progressTimer;
    bool m_isAcquiring;
    bool m_cancelRequested;

    // 拍摄选项
    QRadioButton* m_directCaptureBtn;
    QRadioButton* m_alignCaptureBtn;

    // 当前拍摄点位
    QGroupBox* m_capturePositionGroup;
    QLineEdit* m_captureXEdit;
    QLineEdit* m_captureYEdit;
    QLineEdit* m_captureZEdit;
    QLineEdit* m_captureBEdit;
    QLineEdit* m_captureCEdit;
    QPushButton* m_getCapturePosBtn;

    // 模型原点的机械坐标
    QGroupBox* m_modelOriginGroup;
    QLineEdit* m_modelXEdit;
    QLineEdit* m_modelYEdit;
    QLineEdit* m_modelZEdit;
    QLineEdit* m_modelBEdit;
    QLineEdit* m_modelCEdit;
    QPushButton* m_getModelOriginBtn;
};