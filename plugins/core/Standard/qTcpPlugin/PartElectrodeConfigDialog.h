#pragma once

#include <QDialog>
#include <QListWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>

struct ElectrodeData
{
    QString electrodeName;
    QString processPosition;
    QString dischargeParameter;
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
    void checkUnsavedChanges();

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
        COL_COUNT
    };
};
