#include "PartElectrodeConfigDialog.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QMessageBox>
#include <QInputDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QCloseEvent>
#include <qlabel.h>
#include <qformlayout.h>
#include <ccMainAppInterface.h>
#include <ccHObject.h>

PartElectrodeConfigDialog::PartElectrodeConfigDialog(ccMainAppInterface* app, QWidget* parent)
    : QDialog(parent)
    , m_app(app)
{
    setWindowTitle("工件电极配置");
    setMinimumSize(1000, 600);
    resize(1200, 650);
    initUI();
    loadConfigFromFiles();
}

PartElectrodeConfigDialog::~PartElectrodeConfigDialog()
{
}

void PartElectrodeConfigDialog::initUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(10);

    QHBoxLayout* contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(10);

    m_partList = new QListWidget(this);
    m_partList->setFixedWidth(240);
	m_partList->setSpacing(4);

	m_partList->setStyleSheet(R"(
QListWidget {
    border: 1px solid #dcdcdc;
    border-radius: 6px;
    background: white;
    outline: none;
    padding: 6px;
}

QListWidget::item {
    height: 32px;
    padding-left: 10px;
    border-radius: 4px;
    color: #303030;
}

QListWidget::item:hover {
    background: #f2f6fc;
}

QListWidget::item:selected {
    background: #4a90d9;
    color: white;
}

QListWidget::item:selected:active {
    background: #3d7fc0;
}
)");
    contentLayout->addWidget(m_partList);

    m_electrodeTable = new QTableWidget(this);
    m_electrodeTable->setColumnCount(COL_COUNT);
    QStringList headers = { "电极名称", "加工位置", "放电参数", "扫描位置", "裁剪区域", "开始X", "开始Y", "开始Z", "开始A", "开始B", "开始C" };
    m_electrodeTable->setHorizontalHeaderLabels(headers);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_ELECTRODE, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_POSITION, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_PARAMETER, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_SCAN_POSITION, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_REGION, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_X, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_Y, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_Z, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_A, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_B, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_START_C, QHeaderView::Stretch);
    m_electrodeTable->setColumnWidth(COL_ELECTRODE, 140);
    m_electrodeTable->setColumnWidth(COL_POSITION, 120);
    m_electrodeTable->setColumnWidth(COL_PARAMETER, 120);
    m_electrodeTable->setColumnWidth(COL_SCAN_POSITION, 120);
    m_electrodeTable->setColumnWidth(COL_REGION, 120);
    m_electrodeTable->setColumnWidth(COL_START_X, 90);
    m_electrodeTable->setColumnWidth(COL_START_Y, 90);
    m_electrodeTable->setColumnWidth(COL_START_Z, 90);
    m_electrodeTable->setColumnWidth(COL_START_A, 90);
    m_electrodeTable->setColumnWidth(COL_START_B, 90);
    m_electrodeTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_electrodeTable->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    contentLayout->addWidget(m_electrodeTable);

    mainLayout->addLayout(contentLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);

    QWidget* leftButtons = new QWidget(this);
    QHBoxLayout* leftLayout = new QHBoxLayout(leftButtons);
    leftLayout->setSpacing(8);

    m_addPartBtn = new QPushButton("添加工件", this);
    m_addPartBtn->setFixedSize(100, 32);
    leftLayout->addWidget(m_addPartBtn);

    m_deletePartBtn = new QPushButton("删除工件", this);
    m_deletePartBtn->setFixedSize(100, 32);
    m_deletePartBtn->setEnabled(false);
    leftLayout->addWidget(m_deletePartBtn);

    buttonLayout->addWidget(leftButtons);

    buttonLayout->addStretch();

    QWidget* rightButtons = new QWidget(this);
    QHBoxLayout* rightLayout = new QHBoxLayout(rightButtons);
    rightLayout->setSpacing(8);

    m_addElectrodeBtn = new QPushButton("添加电极", this);
    m_addElectrodeBtn->setFixedSize(120, 32);
    m_addElectrodeBtn->setEnabled(false);
    rightLayout->addWidget(m_addElectrodeBtn);

    m_deleteElectrodeBtn = new QPushButton("删除电极", this);
    m_deleteElectrodeBtn->setFixedSize(140, 32);
    m_deleteElectrodeBtn->setEnabled(false);
    rightLayout->addWidget(m_deleteElectrodeBtn);

    buttonLayout->addWidget(rightButtons);

    buttonLayout->addStretch();

    m_saveBtn = new QPushButton("保存", this);
    m_saveBtn->setFixedSize(80, 32);
    m_saveBtn->setEnabled(false);
    buttonLayout->addWidget(m_saveBtn);

    m_cancelBtn = new QPushButton("取消", this);
    m_cancelBtn->setFixedSize(80, 32);
    buttonLayout->addWidget(m_cancelBtn);

    mainLayout->addLayout(buttonLayout);

	m_hasUnsavedChanges = false;
	updateSaveButtonState();

    connect(m_partList, &QListWidget::itemSelectionChanged, this, &PartElectrodeConfigDialog::onPartSelectionChanged);
    connect(m_electrodeTable, &QTableWidget::itemSelectionChanged, [this]() {
        m_deleteElectrodeBtn->setEnabled(m_electrodeTable->selectedItems().size() > 0);
    });

    connect(m_addPartBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onAddPart);
    connect(m_deletePartBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onDeletePart);
    connect(m_addElectrodeBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onAddElectrode);
    connect(m_deleteElectrodeBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onDeleteElectrode);
    connect(m_saveBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onSave);
    connect(m_cancelBtn, &QPushButton::clicked, this, &PartElectrodeConfigDialog::onCancel);
}

void PartElectrodeConfigDialog::updateSaveButtonState()
{
    m_saveBtn->setEnabled(m_hasUnsavedChanges);
}

void PartElectrodeConfigDialog::loadConfigFromFiles()
{
    QString configDir = getConfigDir();
    QDir dir(configDir);

    if (!dir.exists()) {
        if (!dir.mkpath(configDir)) {
            showErrorMessage("无法创建配置目录: " + configDir);
            return;
        }
        return;
    }

    QStringList filters;
    filters << "*.json";
    dir.setNameFilters(filters);

    QStringList files = dir.entryList(filters, QDir::Files);
    for (const QString& file : files) {
        QString partName = file.left(file.lastIndexOf('.'));
        QString filePath = dir.filePath(file);

        QFile jsonFile(filePath);
        if (!jsonFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            continue;
        }

        QByteArray data = jsonFile.readAll();
        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(data, &error);

        if (error.error != QJsonParseError::NoError) {
            showErrorMessage("JSON 文件解析错误: " + file);
            continue;
        }

        PartData partData;
        partData.partName = doc["partName"].toString();
        if (partData.partName.isEmpty()) {
            partData.partName = partName;
        }

        QJsonArray electrodes = doc["electrodes"].toArray();
        for (const QJsonValue& val : electrodes) {
            QJsonObject obj = val.toObject();
            ElectrodeData electrode;
            electrode.electrodeName = obj["electrodeName"].toString();
            electrode.processPosition = obj["processPosition"].toString();
            electrode.dischargeParameter = obj["dischargeParameter"].toString();
            electrode.startX = obj["startX"].toDouble();
            electrode.startY = obj["startY"].toDouble();
            electrode.startZ = obj["startZ"].toDouble();
            electrode.startA = obj["startA"].toDouble();
            electrode.startB = obj["startB"].toDouble();
            electrode.startC = obj["startC"].toDouble();
            electrode.positionModified = false;
            electrode.parameterModified = false;

            QJsonArray scanPositionsArray = obj["scanPositions"].toArray();
            for (const QJsonValue& scanVal : scanPositionsArray) {
                QJsonObject scanObj = scanVal.toObject();
                ScanPositionData scanPos;
                scanPos.name = scanObj["name"].toString();
                scanPos.x = scanObj["x"].toDouble();
                scanPos.y = scanObj["y"].toDouble();
                scanPos.z = scanObj["z"].toDouble();
                scanPos.b = scanObj["b"].toDouble();
                scanPos.c = scanObj["c"].toDouble();
                electrode.scanPositions.append(scanPos);
            }

            QJsonArray regionsArray = obj["regions"].toArray();
            for (const QJsonValue& regionVal : regionsArray) {
                QJsonObject regionObj = regionVal.toObject();
                RegionData region;
                region.name = regionObj["name"].toString();
                electrode.regions.append(region);
            }

            partData.electrodes.append(electrode);
        }

        partData.isModified = false;
        m_partDataMap[partName] = partData;
        m_partList->addItem(partName);
    }

    if (m_partList->count() > 0) {
        m_partList->setCurrentRow(0);
    }
}

void PartElectrodeConfigDialog::saveConfigToFiles()
{
    QString configDir = getConfigDir();
    QDir dir(configDir);

    if (!dir.exists()) {
        if (!dir.mkpath(configDir)) {
            showErrorMessage("无法创建配置目录: " + configDir);
            return;
        }
    }

    for (auto it = m_partDataMap.begin(); it != m_partDataMap.end(); ++it) {
        const QString& partName = it.key();
        PartData& partData = it.value();

        /*if (!partData.isModified) {
            continue;
        }*/

        QJsonObject obj;
        obj["partName"] = partName;

        QJsonArray electrodesArray;
        for (const ElectrodeData& electrode : partData.electrodes) {
            QJsonObject electrodeObj;
            electrodeObj["electrodeName"] = electrode.electrodeName;
            electrodeObj["processPosition"] = electrode.processPosition;
            electrodeObj["dischargeParameter"] = electrode.dischargeParameter;
            electrodeObj["startX"] = electrode.startX;
            electrodeObj["startY"] = electrode.startY;
            electrodeObj["startZ"] = electrode.startZ;
            electrodeObj["startA"] = electrode.startA;
            electrodeObj["startB"] = electrode.startB;
            electrodeObj["startC"] = electrode.startC;

            QJsonArray scanPositionsArray;
            for (const ScanPositionData& scanPos : electrode.scanPositions) {
                QJsonObject scanPosObj;
                scanPosObj["name"] = scanPos.name;
                scanPosObj["x"] = scanPos.x;
                scanPosObj["y"] = scanPos.y;
                scanPosObj["z"] = scanPos.z;
                scanPosObj["b"] = scanPos.b;
                scanPosObj["c"] = scanPos.c;
                scanPositionsArray.append(scanPosObj);
            }
            electrodeObj["scanPositions"] = scanPositionsArray;

            QJsonArray regionsArray;
            for (const RegionData& region : electrode.regions) {
                QJsonObject regionObj;
                regionObj["name"] = region.name;
                regionsArray.append(regionObj);
            }
            electrodeObj["regions"] = regionsArray;

            electrodesArray.append(electrodeObj);
        }
        obj["electrodes"] = electrodesArray;

        QJsonDocument doc(obj);
        QString filePath = getPartFilePath(partName);

        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            showErrorMessage("无法写入文件: " + filePath);
            return;
        }

        file.write(doc.toJson(QJsonDocument::Indented));
        file.close();

        partData.isModified = false;
    }

    m_hasUnsavedChanges = false;
}

void PartElectrodeConfigDialog::updateElectrodeTable()
{
    disconnect(m_electrodeTable, &QTableWidget::cellDoubleClicked, this, &PartElectrodeConfigDialog::onConfigureScanPosition);
    disconnect(m_electrodeTable, &QTableWidget::cellChanged, this, &PartElectrodeConfigDialog::onCellChanged);

    clearElectrodeTable();

    if (m_currentPartName.isEmpty()) {
        connect(m_electrodeTable, &QTableWidget::cellChanged, this, &PartElectrodeConfigDialog::onCellChanged);
        return;
    }

    if (!m_partDataMap.contains(m_currentPartName)) {
        connect(m_electrodeTable, &QTableWidget::cellChanged, this, &PartElectrodeConfigDialog::onCellChanged);
        return;
    }

    PartData& partData = m_partDataMap[m_currentPartName];
    m_electrodeTable->setRowCount(partData.electrodes.size());

    for (int i = 0; i < partData.electrodes.size(); ++i) {
        const ElectrodeData& electrode = partData.electrodes[i];

        QTableWidgetItem* item0 = new QTableWidgetItem(electrode.electrodeName);
        item0->setFlags(item0->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_ELECTRODE, item0);

        QTableWidgetItem* item1 = new QTableWidgetItem(electrode.processPosition);
        item1->setFlags(item1->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_POSITION, item1);

        QTableWidgetItem* item2 = new QTableWidgetItem(electrode.dischargeParameter);
        item2->setFlags(item2->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_PARAMETER, item2);

        QString scanPosText;
        if (electrode.scanPositions.isEmpty()) {
            scanPosText = "未配置";
        } else {
            scanPosText = QString("%1组扫描点").arg(electrode.scanPositions.size());
        }
        QTableWidgetItem* item3 = new QTableWidgetItem(scanPosText);
        item3->setFlags(item3->flags() & ~Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_SCAN_POSITION, item3);

        QString regionText;
        if (electrode.regions.isEmpty()) {
            regionText = "未配置";
        } else {
            regionText = QString("%1个裁剪区域").arg(electrode.regions.size());
        }
        QTableWidgetItem* item4 = new QTableWidgetItem(regionText);
        item4->setFlags(item4->flags() & ~Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_REGION, item4);

        QTableWidgetItem* item5 = new QTableWidgetItem(QString::number(electrode.startX));
        item5->setFlags(item5->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_X, item5);

        QTableWidgetItem* item6 = new QTableWidgetItem(QString::number(electrode.startY));
        item6->setFlags(item6->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_Y, item6);

        QTableWidgetItem* item7 = new QTableWidgetItem(QString::number(electrode.startZ));
        item7->setFlags(item7->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_Z, item7);

        QTableWidgetItem* item8 = new QTableWidgetItem(QString::number(electrode.startA));
        item8->setFlags(item8->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_A, item8);

        QTableWidgetItem* item9 = new QTableWidgetItem(QString::number(electrode.startB));
        item9->setFlags(item9->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_B, item9);

        QTableWidgetItem* item10 = new QTableWidgetItem(QString::number(electrode.startC));
        item10->setFlags(item10->flags() | Qt::ItemIsEditable);
        m_electrodeTable->setItem(i, COL_START_C, item10);
    }

    m_addElectrodeBtn->setEnabled(true);

    connect(m_electrodeTable, &QTableWidget::cellDoubleClicked, this, &PartElectrodeConfigDialog::onConfigureScanPosition);
    connect(m_electrodeTable, &QTableWidget::cellChanged, this, &PartElectrodeConfigDialog::onCellChanged);
}

void PartElectrodeConfigDialog::clearElectrodeTable()
{
    m_electrodeTable->clearContents();
    m_electrodeTable->setRowCount(0);
}

bool PartElectrodeConfigDialog::validateData()
{
    for (auto it = m_partDataMap.begin(); it != m_partDataMap.end(); ++it) {
        const QString& partName = it.key();
        if (partName.isEmpty()) {
            showErrorMessage("工件名称不能为空");
            return false;
        }

        PartData& partData = it.value();
        QSet<QString> electrodeNames;

        for (const ElectrodeData& electrode : partData.electrodes) {
            if (electrode.electrodeName.isEmpty()) {
                showErrorMessage("电极名称不能为空");
                return false;
            }

            if (electrodeNames.contains(electrode.electrodeName)) {
                showErrorMessage("工件 " + partName + " 中存在重复的电极名称: " + electrode.electrodeName);
                return false;
            }
            electrodeNames.insert(electrode.electrodeName);

            if (electrode.processPosition.isEmpty()) {
                showErrorMessage("电极 " + electrode.electrodeName + " 的加工位置不能为空");
                return false;
            }

            if (electrode.dischargeParameter.isEmpty()) {
                showErrorMessage("电极 " + electrode.electrodeName + " 的放电参数不能为空");
                return false;
            }
        }
    }

    return true;
}

QString PartElectrodeConfigDialog::getConfigDir()
{
    QString appDir = QCoreApplication::applicationDirPath();
    return appDir + "/PartConfig";
}

QString PartElectrodeConfigDialog::getPartFilePath(const QString& partName)
{
    QString configDir = getConfigDir();
    return configDir + "/" + partName + ".json";
}

bool PartElectrodeConfigDialog::isValidFileName(const QString& name)
{
    if (name.isEmpty()) {
        return false;
    }

    QString invalidChars = "\\/:*?\"<>|";
    for (QChar c : invalidChars) {
        if (name.contains(c)) {
            return false;
        }
    }

    return true;
}

void PartElectrodeConfigDialog::showErrorMessage(const QString& message)
{
    QMessageBox::warning(this, "错误", message);
}

bool PartElectrodeConfigDialog::checkUnsavedChanges() 
{
    if (!m_hasUnsavedChanges) {
        return true;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "您修改的数据尚未保存，确定要放弃更改并退出吗？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        if (validateData()) {
            saveConfigToFiles();
            return true;
        }
        return false;
    case QMessageBox::Discard:
        return true;
    case QMessageBox::Cancel:
        return false;
    }
    return false;
}

void PartElectrodeConfigDialog::closeEvent(QCloseEvent* event)
{
    if (checkUnsavedChanges()) {
        event->accept();
    } else {
        event->ignore();
    }
}

void PartElectrodeConfigDialog::onAddPart()
{
    bool ok;
    QString partName = QInputDialog::getText(this, "新增工件", "请输入工件名称:",
        QLineEdit::Normal, "", &ok);

    if (!ok || partName.isEmpty()) {
        return;
    }

    if (!isValidFileName(partName)) {
        showErrorMessage("工件名称包含非法字符: \\/:*?\"<>|");
        return;
    }

    if (m_partDataMap.contains(partName)) {
        showErrorMessage("工件名称已存在: " + partName);
        return;
    }

    PartData partData;
    partData.partName = partName;
    partData.isModified = true;
    m_partDataMap[partName] = partData;

    m_partList->addItem(partName);
    m_partList->setCurrentItem(m_partList->item(m_partList->count() - 1));

    m_hasUnsavedChanges = true;
    updateSaveButtonState();
}

void PartElectrodeConfigDialog::onDeletePart()
{
    if (m_currentPartName.isEmpty()) {
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认删除",
        "是否删除当前工件配置？",
        QMessageBox::Yes | QMessageBox::No);

    if (result != QMessageBox::Yes) {
        return;
    }

    QString filePath = getPartFilePath(m_currentPartName);
    QFile file(filePath);
    if (file.exists()) {
        file.remove();
    }

    m_partDataMap.remove(m_currentPartName);

    QListWidgetItem* currentItem = m_partList->currentItem();
    int currentRow = m_partList->row(currentItem);
    delete currentItem;

    m_currentPartName.clear();
    clearElectrodeTable();
    m_deletePartBtn->setEnabled(false);
    m_addElectrodeBtn->setEnabled(false);

    if (m_partList->count() > 0) {
        if (currentRow >= m_partList->count()) {
            currentRow = m_partList->count() - 1;
        }
        QListWidgetItem* newItem = m_partList->item(currentRow);
        if (newItem) {
            m_currentPartName = newItem->text();
            m_partList->setCurrentItem(newItem);
            m_deletePartBtn->setEnabled(true);
            m_addElectrodeBtn->setEnabled(true);
            updateElectrodeTable();
        }
    }

    m_hasUnsavedChanges = true;
    updateSaveButtonState();
}

void PartElectrodeConfigDialog::onAddElectrode()
{
    if (m_currentPartName.isEmpty()) {
        return;
    }

    bool ok;
    QString electrodeName = QInputDialog::getText(this, "添加电极", "请输入电极名称:",
        QLineEdit::Normal, "", &ok);

    if (!ok || electrodeName.isEmpty()) {
        return;
    }

    PartData& partData = m_partDataMap[m_currentPartName];

    for (const ElectrodeData& electrode : partData.electrodes) {
        if (electrode.electrodeName == electrodeName) {
            showErrorMessage("电极名称已存在: " + electrodeName);
            return;
        }
    }

    ElectrodeData electrode;
    electrode.electrodeName = electrodeName;
    electrode.processPosition = electrodeName + "_ROW1";
    electrode.dischargeParameter = electrodeName + "_CS1";
    electrode.positionModified = false;
    electrode.parameterModified = false;

    partData.electrodes.append(electrode);
    partData.isModified = true;
    m_hasUnsavedChanges = true;
    updateSaveButtonState();

    updateElectrodeTable();
}

void PartElectrodeConfigDialog::onDeleteElectrode()
{
    int currentRow = m_electrodeTable->currentRow();
    if (currentRow < 0) {
        return;
    }

    if (m_currentPartName.isEmpty()) {
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认删除",
        "确定要删除选中的电极吗？",
        QMessageBox::Yes | QMessageBox::No);

    if (result != QMessageBox::Yes) {
        return;
    }

    PartData& partData = m_partDataMap[m_currentPartName];
    partData.electrodes.removeAt(currentRow);
    partData.isModified = true;
    m_hasUnsavedChanges = true;
    updateSaveButtonState();

    updateElectrodeTable();
}

void PartElectrodeConfigDialog::onPartSelectionChanged()
{
    QListWidgetItem* item = m_partList->currentItem();
    if (!item) {
        m_currentPartName.clear();
        clearElectrodeTable();
        m_deletePartBtn->setEnabled(false);
        m_addElectrodeBtn->setEnabled(false);
        return;
    }

    m_currentPartName = item->text();
    m_deletePartBtn->setEnabled(true);
    m_addElectrodeBtn->setEnabled(true);

    updateElectrodeTable();
}

void PartElectrodeConfigDialog::onCellChanged(int row, int column)
{
    if (m_currentPartName.isEmpty()) {
        return;
    }

    PartData& partData = m_partDataMap[m_currentPartName];
    if (row >= partData.electrodes.size()) {
        return;
    }

    ElectrodeData& electrode = partData.electrodes[row];
    QString oldValue;
    QString newValue = m_electrodeTable->item(row, column)->text();

    switch (column) {
    case COL_ELECTRODE:
        oldValue = electrode.electrodeName;
        electrode.electrodeName = newValue;

        if (true || !electrode.positionModified) {
            electrode.processPosition = newValue + "_ROW1";
            QTableWidgetItem* posItem = m_electrodeTable->item(row, COL_POSITION);
            if (posItem) {
                posItem->setText(electrode.processPosition);
            }
        }

        if (true || !electrode.parameterModified)
		{
            electrode.dischargeParameter = newValue + "_CS1";
            QTableWidgetItem* paramItem = m_electrodeTable->item(row, COL_PARAMETER);
            if (paramItem) {
                paramItem->setText(electrode.dischargeParameter);
            }
        }
        break;

    case COL_POSITION:
        electrode.processPosition = newValue;
        electrode.positionModified = true;
        break;

    case COL_PARAMETER:
        electrode.dischargeParameter = newValue;
        electrode.parameterModified = true;
        break;

    case COL_START_X:
        electrode.startX = newValue.toDouble();
        break;

    case COL_START_Y:
        electrode.startY = newValue.toDouble();
        break;

    case COL_START_Z:
        electrode.startZ = newValue.toDouble();
        break;

    case COL_START_A:
        electrode.startA = newValue.toDouble();
        break;

    case COL_START_B:
        electrode.startB = newValue.toDouble();
        break;

    case COL_START_C:
        electrode.startC = newValue.toDouble();
        break;
    }

    partData.isModified = true;
    m_hasUnsavedChanges = true;
    updateSaveButtonState();
}

void PartElectrodeConfigDialog::onSave()
{
    if (!validateData()) {
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认保存",
        "确定要保存所有配置吗？",
        QMessageBox::Yes | QMessageBox::No);

    if (result != QMessageBox::Yes) {
        return;
    }

    saveConfigToFiles();
	updateSaveButtonState();
}

void PartElectrodeConfigDialog::onCancel()
{
    if (checkUnsavedChanges()) {
        reject();
    }
}

void PartElectrodeConfigDialog::onConfigureScanPosition(int row, int column)
{
    if (m_currentPartName.isEmpty()) {
        return;
    }

    PartData& partData = m_partDataMap[m_currentPartName];
    if (row >= partData.electrodes.size()) {
        return;
    }

    if (column == COL_SCAN_POSITION) {
        QList<ScanPositionData> originalScanPositions = partData.electrodes[row].scanPositions;

        ScanPositionConfigDialog dlg(partData.electrodes[row].scanPositions, this, this);
        int result = dlg.exec();

        if (result == QDialog::Rejected && dlg.hasChanges()) {
            partData.electrodes[row].scanPositions = originalScanPositions;
        }
    } else if (column == COL_REGION) {
        QList<RegionData> originalRegions = partData.electrodes[row].regions;

        RegionConfigDialog dlg(partData.electrodes[row].regions, m_app, this, this);
        int result = dlg.exec();

        if (result == QDialog::Rejected && dlg.hasChanges()) {
            partData.electrodes[row].regions = originalRegions;
        }
    }

    updateElectrodeTable();
}

PartElectrodeConfigDialog::ScanPositionConfigDialog::ScanPositionConfigDialog(QList<ScanPositionData>& scanPositions, PartElectrodeConfigDialog* parentDialog, QWidget* parent)
    : QDialog(parent), m_scanPositions(scanPositions), m_parentDialog(parentDialog)
{
    setWindowTitle("配置扫描位置");
    setMinimumSize(600, 450);
    resize(650, 500);
    initUI();
    updateScanPositionList();
}

PartElectrodeConfigDialog::ScanPositionConfigDialog::~ScanPositionConfigDialog()
{
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::closeEvent(QCloseEvent* event)
{
    if (!m_hasChanges) {
        event->accept();
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "您修改的数据尚未保存，确定要放弃更改并退出吗？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        m_hasChanges = false;
        m_saveBtn->setEnabled(false);
        event->accept();
        break;
    case QMessageBox::Discard:
        event->accept();
        break;
    case QMessageBox::Cancel:
        event->ignore();
        break;
    }
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::initUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(10);

    QHBoxLayout* contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(10);

    m_scanPositionList = new QListWidget(this);
    m_scanPositionList->setFixedWidth(180);
	m_scanPositionList->setSpacing(4);
	m_scanPositionList->setStyleSheet(R"(
QListWidget {
    border: 1px solid #dcdcdc;
    border-radius: 6px;
    background: white;
    outline: none;
    padding: 6px;
}

QListWidget::item {
    height: 32px;
    padding-left: 10px;
    border-radius: 4px;
    color: #303030;
}

QListWidget::item:hover {
    background: #f2f6fc;
}

QListWidget::item:selected {
    background: #4a90d9;
    color: white;
}

QListWidget::item:selected:active {
    background: #3d7fc0;
}
)");

    contentLayout->addWidget(m_scanPositionList);

    QWidget* editPanel = new QWidget(this);
    QVBoxLayout* editLayout = new QVBoxLayout(editPanel);
    editLayout->setContentsMargins(0, 0, 0, 0);
    editLayout->setSpacing(10);

    QLabel* coordLabel = new QLabel("坐标参数", this);
    coordLabel->setStyleSheet("font-weight: bold;");
    editLayout->addWidget(coordLabel);

    QFormLayout* formLayout = new QFormLayout();
    formLayout->setSpacing(8);

    m_xEdit = new QLineEdit(this);
    m_xEdit->setFixedWidth(150);
    formLayout->addRow("X:", m_xEdit);

    m_yEdit = new QLineEdit(this);
    m_yEdit->setFixedWidth(150);
    formLayout->addRow("Y:", m_yEdit);

    m_zEdit = new QLineEdit(this);
    m_zEdit->setFixedWidth(150);
    formLayout->addRow("Z:", m_zEdit);

    m_bEdit = new QLineEdit(this);
    m_bEdit->setFixedWidth(150);
    formLayout->addRow("B:", m_bEdit);

    m_cEdit = new QLineEdit(this);
    m_cEdit->setFixedWidth(150);
    formLayout->addRow("C:", m_cEdit);

    editLayout->addLayout(formLayout);
    editLayout->addStretch();

    contentLayout->addWidget(editPanel);
    mainLayout->addLayout(contentLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);

    m_addBtn = new QPushButton("添加扫描点", this);
    m_addBtn->setFixedSize(100, 32);
    buttonLayout->addWidget(m_addBtn);

    m_deleteBtn = new QPushButton("删除扫描点", this);
    m_deleteBtn->setFixedSize(100, 32);
    m_deleteBtn->setEnabled(false);
    buttonLayout->addWidget(m_deleteBtn);

    m_copyBtn = new QPushButton("复制扫描点", this);
    m_copyBtn->setFixedSize(100, 32);
    m_copyBtn->setEnabled(false);
    buttonLayout->addWidget(m_copyBtn);

    m_renameBtn = new QPushButton("重命名", this);
    m_renameBtn->setFixedSize(80, 32);
    m_renameBtn->setEnabled(false);
    buttonLayout->addWidget(m_renameBtn);

    buttonLayout->addStretch();

    m_saveBtn = new QPushButton("保存", this);
    m_saveBtn->setFixedSize(80, 32);
    m_saveBtn->setEnabled(false);
    buttonLayout->addWidget(m_saveBtn);

    m_cancelBtn = new QPushButton("取消", this);
    m_cancelBtn->setFixedSize(80, 32);
    buttonLayout->addWidget(m_cancelBtn);

    mainLayout->addLayout(buttonLayout);

    connect(m_scanPositionList, &QListWidget::itemSelectionChanged, this, &ScanPositionConfigDialog::onSelectionChanged);
    connect(m_addBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onAddScanPosition);
    connect(m_deleteBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onDeleteScanPosition);
    connect(m_copyBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onCopyScanPosition);
    connect(m_renameBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onRenameScanPosition);
    connect(m_saveBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onSave);
    connect(m_cancelBtn, &QPushButton::clicked, this, &ScanPositionConfigDialog::onCancel);

    connect(m_xEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_yEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_zEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_bEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_cEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::updateScanPositionList()
{
    m_scanPositionList->clear();
    for (const ScanPositionData& pos : m_scanPositions) {
        m_scanPositionList->addItem(pos.name);
    }
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::updateCoordinateFields()
{
    disconnect(m_xEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    disconnect(m_yEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    disconnect(m_zEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    disconnect(m_bEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    disconnect(m_cEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);

    QListWidgetItem* item = m_scanPositionList->currentItem();
    if (!item) {
        m_xEdit->clear();
        m_yEdit->clear();
        m_zEdit->clear();
        m_bEdit->clear();
        m_cEdit->clear();
        return;
    }

    int index = m_scanPositionList->row(item);
    if (index >= 0 && index < m_scanPositions.size()) {
        const ScanPositionData& pos = m_scanPositions[index];
        m_xEdit->setText(QString::number(pos.x));
        m_yEdit->setText(QString::number(pos.y));
        m_zEdit->setText(QString::number(pos.z));
        m_bEdit->setText(QString::number(pos.b));
        m_cEdit->setText(QString::number(pos.c));
    }

    connect(m_xEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_yEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_zEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_bEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
    connect(m_cEdit, &QLineEdit::textChanged, this, &ScanPositionConfigDialog::onCoordinateChanged);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onAddScanPosition()
{
    int newIndex = m_scanPositions.size() + 1;
    QString defaultName = QString("扫描点%1").arg(newIndex);

    ScanPositionData pos;
    pos.name = defaultName;
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    pos.b = 0.0;
    pos.c = 0.0;

    m_scanPositions.append(pos);
    updateScanPositionList();

    m_scanPositionList->setCurrentRow(m_scanPositions.size() - 1);
    m_hasChanges = true;
    m_saveBtn->setEnabled(true);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onDeleteScanPosition()
{
    QListWidgetItem* item = m_scanPositionList->currentItem();
    if (!item) {
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认删除",
        "确定要删除选中的扫描点吗？",
        QMessageBox::Yes | QMessageBox::No);

    if (result != QMessageBox::Yes) {
        return;
    }

    int index = m_scanPositionList->row(item);
    m_scanPositions.removeAt(index);
    updateScanPositionList();

    if (m_scanPositions.size() > 0) {
        if (index >= m_scanPositions.size()) {
            index = m_scanPositions.size() - 1;
        }
        m_scanPositionList->setCurrentRow(index);
    }

    m_hasChanges = true;
    m_saveBtn->setEnabled(true);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onCopyScanPosition()
{
    QListWidgetItem* item = m_scanPositionList->currentItem();
    if (!item) {
        return;
    }

    int index = m_scanPositionList->row(item);
    if (index < 0 || index >= m_scanPositions.size()) {
        return;
    }

    ScanPositionData source = m_scanPositions[index];
    QString baseName = source.name;
    QString newName = baseName + "_副本";
    int counter = 1;

    while (true) {
        bool exists = false;
        for (const ScanPositionData& pos : m_scanPositions) {
            if (pos.name == newName) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            break;
        }
        newName = QString("%1_副本%2").arg(baseName).arg(counter++);
    }

    ScanPositionData copy = source;
    copy.name = newName;
    m_scanPositions.insert(index + 1, copy);
    updateScanPositionList();
    m_scanPositionList->setCurrentRow(index + 1);

    m_hasChanges = true;
    m_saveBtn->setEnabled(true);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onRenameScanPosition()
{
    QListWidgetItem* item = m_scanPositionList->currentItem();
    if (!item) {
        return;
    }

    bool ok;
    QString newName = QInputDialog::getText(this, "重命名", "请输入新名称:",
        QLineEdit::Normal, item->text(), &ok);

    if (!ok || newName.isEmpty()) {
        return;
    }

    for (const ScanPositionData& pos : m_scanPositions) {
        if (pos.name == newName && pos.name != item->text()) {
            QMessageBox::warning(this, "错误", "名称已存在");
            return;
        }
    }

    int index = m_scanPositionList->row(item);
    if (index >= 0 && index < m_scanPositions.size()) {
        m_scanPositions[index].name = newName;
        updateScanPositionList();
        m_scanPositionList->setCurrentRow(index);

        m_hasChanges = true;
        m_saveBtn->setEnabled(true);
    }
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onSelectionChanged()
{
    bool hasSelection = m_scanPositionList->selectedItems().size() > 0;
    m_deleteBtn->setEnabled(hasSelection);
    m_copyBtn->setEnabled(hasSelection);
    m_renameBtn->setEnabled(hasSelection);
    updateCoordinateFields();
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onCoordinateChanged()
{
    QListWidgetItem* item = m_scanPositionList->currentItem();
    if (!item) {
        return;
    }

    int index = m_scanPositionList->row(item);
    if (index >= 0 && index < m_scanPositions.size()) {
        ScanPositionData& pos = m_scanPositions[index];
        pos.x = m_xEdit->text().toDouble();
        pos.y = m_yEdit->text().toDouble();
        pos.z = m_zEdit->text().toDouble();
        pos.b = m_bEdit->text().toDouble();
        pos.c = m_cEdit->text().toDouble();

        m_hasChanges = true;
        m_saveBtn->setEnabled(true);
    }
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onSave()
{
    if (m_parentDialog) {
        m_parentDialog->saveConfigToFiles();
        m_parentDialog->resetSavedState();
    }
    m_hasChanges = false;
    m_saveBtn->setEnabled(false);
}

void PartElectrodeConfigDialog::ScanPositionConfigDialog::onCancel()
{
    if (!m_hasChanges) {
        reject();
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "您修改的数据尚未保存，确定要放弃更改并退出吗？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        m_hasChanges = false;
        m_saveBtn->setEnabled(false);
        reject();
        break;
    case QMessageBox::Discard:
        reject();
        break;
    case QMessageBox::Cancel:
        break;
    }
}

PartElectrodeConfigDialog::RegionConfigDialog::RegionConfigDialog(QList<RegionData>& regions, ccMainAppInterface* app, PartElectrodeConfigDialog* parentDialog, QWidget* parent)
    : QDialog(parent), m_regions(regions), m_app(app), m_parentDialog(parentDialog)
{
    setWindowTitle("配置裁剪区域");
    setMinimumSize(600, 450);
    resize(650, 500);
    initUI();
    updateRegionList();
    updateAvailableRegions();
}

PartElectrodeConfigDialog::RegionConfigDialog::~RegionConfigDialog()
{
}

void PartElectrodeConfigDialog::RegionConfigDialog::initUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(10);

    QHBoxLayout* contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(10);

    QWidget* leftWidget = new QWidget(this);
    QVBoxLayout* leftLayout = new QVBoxLayout(leftWidget);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(8);

    QLabel* availableLabel = new QLabel("CloudCompare 中的裁剪区域", this);
    availableLabel->setStyleSheet("font-weight: bold;");
    leftLayout->addWidget(availableLabel);

    m_availableRegionsList = new QListWidget(this);
    m_availableRegionsList->setFixedWidth(280);
    m_availableRegionsList->setStyleSheet(R"(
QListWidget {
    border: 1px solid #dcdcdc;
    border-radius: 6px;
    background: white;
    outline: none;
    padding: 6px;
}
QListWidget::item {
    height: 32px;
    padding-left: 10px;
    border-radius: 4px;
    color: #303030;
}
QListWidget::item:hover {
    background: #f2f6fc;
}
QListWidget::item:selected {
    background: #4a90d9;
    color: white;
}
)");
    leftLayout->addWidget(m_availableRegionsList);

    contentLayout->addWidget(leftWidget);

    QWidget* centerWidget = new QWidget(this);
    QVBoxLayout* centerLayout = new QVBoxLayout(centerWidget);
    centerLayout->setContentsMargins(0, 0, 0, 0);
    centerLayout->setSpacing(8);
    centerLayout->setAlignment(Qt::AlignCenter);

    m_addBtn = new QPushButton(">", this);
    m_addBtn->setFixedSize(40, 32);
    m_addBtn->setEnabled(false);
    centerLayout->addWidget(m_addBtn);

    m_removeBtn = new QPushButton("<", this);
    m_removeBtn->setFixedSize(40, 32);
    m_removeBtn->setEnabled(false);
    centerLayout->addWidget(m_removeBtn);

    contentLayout->addWidget(centerWidget);

    QWidget* rightWidget = new QWidget(this);
    QVBoxLayout* rightLayout = new QVBoxLayout(rightWidget);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(8);

    QLabel* selectedLabel = new QLabel("已选择的裁剪区域", this);
    selectedLabel->setStyleSheet("font-weight: bold;");
    rightLayout->addWidget(selectedLabel);

    m_selectedRegionsList = new QListWidget(this);
    m_selectedRegionsList->setFixedWidth(280);
    m_selectedRegionsList->setStyleSheet(R"(
QListWidget {
    border: 1px solid #dcdcdc;
    border-radius: 6px;
    background: white;
    outline: none;
    padding: 6px;
}
QListWidget::item {
    height: 32px;
    padding-left: 10px;
    border-radius: 4px;
    color: #303030;
}
QListWidget::item:hover {
    background: #f2f6fc;
}
QListWidget::item:selected {
    background: #4a90d9;
    color: white;
}
)");
    rightLayout->addWidget(m_selectedRegionsList);

    contentLayout->addWidget(rightWidget);

    mainLayout->addLayout(contentLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);

    buttonLayout->addStretch();

    m_saveBtn = new QPushButton("保存", this);
    m_saveBtn->setFixedSize(80, 32);
    m_saveBtn->setEnabled(false);
    buttonLayout->addWidget(m_saveBtn);

    m_cancelBtn = new QPushButton("取消", this);
    m_cancelBtn->setFixedSize(80, 32);
    buttonLayout->addWidget(m_cancelBtn);

    mainLayout->addLayout(buttonLayout);

    connect(m_availableRegionsList, &QListWidget::itemSelectionChanged, this, &RegionConfigDialog::onSelectionChanged);
    connect(m_selectedRegionsList, &QListWidget::itemSelectionChanged, this, &RegionConfigDialog::onSelectionChanged);
    connect(m_addBtn, &QPushButton::clicked, this, &RegionConfigDialog::onAddRegion);
    connect(m_removeBtn, &QPushButton::clicked, this, &RegionConfigDialog::onRemoveRegion);
    connect(m_saveBtn, &QPushButton::clicked, this, &RegionConfigDialog::onSave);
    connect(m_cancelBtn, &QPushButton::clicked, this, &RegionConfigDialog::onCancel);
}

void PartElectrodeConfigDialog::RegionConfigDialog::updateRegionList()
{
    m_selectedRegionsList->clear();
    for (const RegionData& region : m_regions) {
        m_selectedRegionsList->addItem(region.name);
    }
}

void PartElectrodeConfigDialog::RegionConfigDialog::updateAvailableRegions()
{
    m_availableRegionsList->clear();

    if (!m_app) {
        return;
    }

    ccHObject* dbRoot = m_app->dbRootObject();
    if (!dbRoot) {
        return;
    }

    QSet<QString> selectedNames;
    for (const RegionData& region : m_regions) {
        selectedNames.insert(region.name);
    }

    for (unsigned i = 0; i < dbRoot->getChildrenNumber(); ++i) {
        ccHObject* child = dbRoot->getChild(i);
		
        if (child && child->isKindOf(CC_TYPES::POLY_LINE))
		{
            QString name = child->getName();

            if ( !selectedNames.contains(name)) {
                m_availableRegionsList->addItem(name);
            }
        }
    }
}

void PartElectrodeConfigDialog::RegionConfigDialog::onAddRegion()
{
    QListWidgetItem* item = m_availableRegionsList->currentItem();
    if (!item) {
        return;
    }

    QString regionName = item->text();

    RegionData region;
    region.name = regionName;
    m_regions.append(region);

    m_hasChanges = true;
    m_saveBtn->setEnabled(true);

    updateRegionList();
    updateAvailableRegions();
}

void PartElectrodeConfigDialog::RegionConfigDialog::onRemoveRegion()
{
    int currentRow = m_selectedRegionsList->currentRow();
    if (currentRow < 0) {
        return;
    }

    m_regions.removeAt(currentRow);

    m_hasChanges = true;
    m_saveBtn->setEnabled(true);

    updateRegionList();
    updateAvailableRegions();
}

void PartElectrodeConfigDialog::RegionConfigDialog::onSelectionChanged()
{
    m_addBtn->setEnabled(m_availableRegionsList->selectedItems().size() > 0);
    m_removeBtn->setEnabled(m_selectedRegionsList->selectedItems().size() > 0);
}

void PartElectrodeConfigDialog::RegionConfigDialog::onSave()
{
    if (m_parentDialog) {
        m_parentDialog->saveConfigToFiles();
        m_parentDialog->resetSavedState();
    }
    m_hasChanges = false;
    m_saveBtn->setEnabled(false);
}

void PartElectrodeConfigDialog::RegionConfigDialog::onCancel()
{
    if (!m_hasChanges) {
        reject();
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "您修改的数据尚未保存，确定要放弃更改并退出吗？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        m_hasChanges = false;
        m_saveBtn->setEnabled(false);
        reject();
        break;
    case QMessageBox::Discard:
        reject();
        break;
    case QMessageBox::Cancel:
        break;
    }
}

void PartElectrodeConfigDialog::RegionConfigDialog::closeEvent(QCloseEvent* event)
{
    if (!m_hasChanges) {
        event->accept();
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "您修改的数据尚未保存，确定要放弃更改并退出吗？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        m_hasChanges = false;
        m_saveBtn->setEnabled(false);
        event->accept();
        break;
    case QMessageBox::Discard:
        event->accept();
        break;
    case QMessageBox::Cancel:
        event->ignore();
        break;
    }
}
