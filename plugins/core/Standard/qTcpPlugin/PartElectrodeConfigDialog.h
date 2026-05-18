#pragma once

#include <QDialog>
#include <QListWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>

struct ScanPositionData
{
    QString name;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct ElectrodeData
{
    QString electrodeName;
    QString processPosition;
    QString dischargeParameter;
    double startX = 0.0;
    double startY = 0.0;
    double startZ = 0.0;
    double startA = 0.0;
    double startB = 0.0;
    double startC = 0.0;
    QList<ScanPositionData> scanPositions;
    bool positionModified = false;
    bool parameterModified = false;
};

struct PartData
{
    QString partName;
    QList<ElectrodeData> electrodes;
    bool isModified = false;
};

class PartElectrodeConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PartElectrodeConfigDialog(QWidget* parent = nullptr);
    ~PartElectrodeConfigDialog() override;
    
    void resetSavedState() {
        m_hasUnsavedChanges = false;
        updateSaveButtonState();
    }

private:
    void initUI();
    void loadConfigFromFiles();
    void saveConfigToFiles();
    void updateElectrodeTable();
    void clearElectrodeTable();
    bool validateData();
    QString getConfigDir();
    QString getPartFilePath(const QString& partName);
    bool isValidFileName(const QString& name);
    void showErrorMessage(const QString& message);
    bool checkUnsavedChanges();
    void updateSaveButtonState();

    void onAddPart();
    void onDeletePart();
    void onAddElectrode();
    void onDeleteElectrode();
    void onPartSelectionChanged();
    void onCellChanged(int row, int column);
    void onSave();
    void onCancel();

	void closeEvent(QCloseEvent* event);

  private:
    QListWidget* m_partList;
    QTableWidget* m_electrodeTable;
    QPushButton* m_addPartBtn;
    QPushButton* m_deletePartBtn;
    QPushButton* m_addElectrodeBtn;
    QPushButton* m_deleteElectrodeBtn;
    QPushButton* m_saveBtn;
    QPushButton* m_cancelBtn;

    QMap<QString, PartData> m_partDataMap;
    QString m_currentPartName;
    bool m_hasUnsavedChanges = false;

    enum ColumnIndex
    {
        COL_ELECTRODE = 0,
        COL_POSITION,
        COL_PARAMETER,
        COL_SCAN_POSITION,
        COL_START_X,
        COL_START_Y,
        COL_START_Z,
        COL_START_A,
        COL_START_B,
        COL_START_C,
        COL_COUNT
    };

private slots:
    void onConfigureScanPosition(int row, int column);

private:
    class ScanPositionConfigDialog : public QDialog
    {
    public:
        explicit ScanPositionConfigDialog(QList<ScanPositionData>& scanPositions, PartElectrodeConfigDialog* parentDialog, QWidget* parent = nullptr);
        ~ScanPositionConfigDialog() override;
        
        bool hasChanges() const { return m_hasChanges; }
        
        void closeEvent(QCloseEvent* event) override;
        
    private:
        void initUI();
        void updateScanPositionList();
        void updateCoordinateFields();
        void onAddScanPosition();
        void onDeleteScanPosition();
        void onCopyScanPosition();
        void onRenameScanPosition();
        void onSelectionChanged();
        void onCoordinateChanged();
        void onSave();
        void onCancel();
        
        QList<ScanPositionData>& m_scanPositions;
        PartElectrodeConfigDialog* m_parentDialog;
        QListWidget* m_scanPositionList;
        QLineEdit* m_xEdit;
        QLineEdit* m_yEdit;
        QLineEdit* m_zEdit;
        QLineEdit* m_bEdit;
        QLineEdit* m_cEdit;
        QPushButton* m_addBtn;
        QPushButton* m_deleteBtn;
        QPushButton* m_copyBtn;
        QPushButton* m_renameBtn;
        QPushButton* m_saveBtn;
        QPushButton* m_cancelBtn;
        bool m_hasChanges = false;
    };
};
