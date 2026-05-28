#pragma once

#include <ccHObject.h>
#include <QDialog>
#include <QListWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>

class ccMainAppInterface;

struct ScanPositionData
{
    QString name;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double b = 0.0;
    double c = 0.0;
};

struct RegionData
{
    QString name;
};

struct ElectrodeData
{
    QString electrodeName;
    QString processPosition;
    QString dischargeParameter;
    QString inspectionProgramPath;
    QString measureMethod;
    QString probeProgramPath;
    double startX = 0.0;
    double startY = 0.0;
    double startZ = 0.0;
    double startA = 0.0;
    double startB = 0.0;
    double startC = 0.0;
    QList<ScanPositionData> scanPositions;
    QList<RegionData> regions;
    bool positionModified = false;
    bool parameterModified = false;
};

struct PartData
{
    QString partName;
    QString modelFilePath;
    QList<ElectrodeData> electrodes;
    bool isModified = false;
    double zeroX = 0.0;
    double zeroY = 0.0;
    double zeroZ = 0.0;
    double zeroB = 0.0;
    double zeroC = 0.0;
};

class PointCloudService;

class PartElectrodeConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PartElectrodeConfigDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
    ~PartElectrodeConfigDialog() override;
    
    void resetSavedState() {
        m_hasUnsavedChanges = false;
        updateSaveButtonState();
    }

    PartData* getCurrentPartData() {
        if (m_partDataMap.contains(m_currentPartName)) {
            return &m_partDataMap[m_currentPartName];
        }
        return nullptr;
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
    void updatePartInfo();
    void onSelectInspectionProgram(int row);
    void onSelectProbeProgram(int row);
    void onMeasureMethodChanged(int row);
    void updateColumnEnabledState(int row, const QString& measureMethod);

    void onAddPart();
    void onDeletePart();
    void onAddElectrode();
    void onDeleteElectrode();
    void onPartSelectionChanged();
    void onCellChanged(int row, int column);
    void onSave();
    void onCancel();
    void onChangeModelFile();
    void onEditPart();

	void closeEvent(QCloseEvent* event);

  private:
    QWidget* m_partInfoWidget;
    QLabel* m_partNameLabel;
    QLabel* m_modelFileLabel;
    QLabel* m_zeroPosLabel;
    QPushButton* m_editPartBtn;
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
    ccMainAppInterface* m_app;
    PointCloudService* m_pointCloudService;

    enum ColumnIndex
    {
        COL_ELECTRODE = 0,
        COL_POSITION,
        COL_PARAMETER,
        COL_INSPECTION_PROGRAM,
        COL_MEASURE_METHOD,
        COL_SCAN_POSITION,
        COL_REGION,
        COL_PROBE_PROGRAM,
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
    void onCellClicked(int row, int column);
    void onCellEntered(int row, int column);

private:
    class AddPartDialog : public QDialog
    {
    public:
        explicit AddPartDialog(QWidget* parent = nullptr);
        QString getPartName() const { return m_partName; }
        QString getModelFilePath() const { return m_modelFilePath; }
        bool isValid() const { return m_valid; }
        double getZeroX() const { return m_zeroX; }
        double getZeroY() const { return m_zeroY; }
        double getZeroZ() const { return m_zeroZ; }
        double getZeroB() const { return m_zeroB; }
        double getZeroC() const { return m_zeroC; }
        void setEditMode(const QString& partName, const QString& modelFilePath, 
                         double zeroX, double zeroY, double zeroZ, double zeroB, double zeroC);

    private:
        QString m_partName;
        QString m_modelFilePath;
        bool m_valid = false;
        bool m_isEditMode = false;
        double m_zeroX = 0.0;
        double m_zeroY = 0.0;
        double m_zeroZ = 0.0;
        double m_zeroB = 0.0;
        double m_zeroC = 0.0;
        QLineEdit* m_nameEdit;
        QLabel* m_filePathLabel;
        QLineEdit* m_zeroXEdit;
        QLineEdit* m_zeroYEdit;
        QLineEdit* m_zeroZEdit;
        QLineEdit* m_zeroBEdit;
        QLineEdit* m_zeroCEdit;
		QPushButton* m_okBtn;

        void onNameChanged(const QString& text);
        void onSelectFile();
		void onGetDeviceCoordinate();
        void onOk();
        void updateOkButton();
    };

    class ScanPositionConfigDialog : public QDialog
    {
    public:
        explicit ScanPositionConfigDialog(ElectrodeData* electrode, PartElectrodeConfigDialog* parentDialog, QWidget* parent = nullptr);
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
        void onGetDeviceCoordinate();
        void onSave();
        void onCancel();
        
        ElectrodeData* m_electrode;
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
        QPushButton* m_getDeviceCoorBtn;
        QPushButton* m_saveBtn;
        QPushButton* m_cancelBtn;
        bool m_hasChanges = false;
        
        PointCloudService* getPointCloudService() const;
    };

    class RegionConfigDialog : public QDialog
    {
    public:
        explicit RegionConfigDialog(QList<RegionData>& regions, ccMainAppInterface* app, PartElectrodeConfigDialog* parentDialog,const QString& partName, const QString& electrodeName, QWidget* parent = nullptr);
        ~RegionConfigDialog() override;
        
        bool hasChanges() const { return m_hasChanges; }
        
        void closeEvent(QCloseEvent* event) override;
        
    private:
        void initUI();
        void updateRegionList();
        void updateAvailableRegions();
        void addRegionsRecursively(ccHObject* parent, const QSet<QString>& selectedNames);
        ccHObject* findObjectRecursively(ccHObject* parent, const QString& name);
        void onAddRegion();
        void onRemoveRegion();
        void onSelectionChanged();
        void onSave();
        void onCancel();
        void saveRegionToFile(const QString& regionName);
        void saveRegionToFileWithCustomName(const QString& originalName, const QString& newName);
        void deleteRegionFile(const QString& regionName);
        bool loadRegionFromFile(const QString& regionName);
        
        QList<RegionData>& m_regions;
        ccMainAppInterface* m_app;
        PartElectrodeConfigDialog* m_parentDialog;
        QString m_partName;
        QString m_electrodeName;
        QListWidget* m_selectedRegionsList;
        QListWidget* m_availableRegionsList;
        QPushButton* m_addBtn;
        QPushButton* m_removeBtn;
        QPushButton* m_saveBtn;
        QPushButton* m_cancelBtn;
        bool m_hasChanges = false;
    };
};
