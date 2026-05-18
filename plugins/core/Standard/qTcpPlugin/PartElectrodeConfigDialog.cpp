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

PartElectrodeConfigDialog::PartElectrodeConfigDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("工件电极配置");
    setMinimumSize(1000, 600);
    resize(1100, 650);
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
    m_partList->setStyleSheet(R"(
        QListWidget {
            border: 1px solid #d0d0d0;
            border-radius: 4px;
        }
        QListWidget::item {
            padding: 8px 12px;
        }
        QListWidget::item:selected {
            background-color: #4a90d9;
            color: white;
        }
    )");
    contentLayout->addWidget(m_partList);

    m_electrodeTable = new QTableWidget(this);
    m_electrodeTable->setColumnCount(COL_COUNT);
    QStringList headers = { "电极名称", "加工位置", "放电参数" };
    m_electrodeTable->setHorizontalHeaderLabels(headers);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_ELECTRODE, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_POSITION, QHeaderView::Fixed);
    m_electrodeTable->horizontalHeader()->setSectionResizeMode(COL_PARAMETER, QHeaderView::Stretch);
    m_electrodeTable->setColumnWidth(COL_ELECTRODE, 220);
    m_electrodeTable->setColumnWidth(COL_POSITION, 260);
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
    buttonLayout->addWidget(m_saveBtn);

    m_cancelBtn = new QPushButton("取消", this);
    m_cancelBtn->setFixedSize(80, 32);
    buttonLayout->addWidget(m_cancelBtn);

    mainLayout->addLayout(buttonLayout);

    connect(m_partList, &QListWidget::itemSelectionChanged, this, &PartElectrodeConfigDialog::onPartSelectionChanged);
    connect(m_electrodeTable, &QTableWidget::cellChanged, this, &PartElectrodeConfigDialog::onCellChanged);
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
            electrode.positionModified = false;
            electrode.parameterModified = false;
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

        if (!partData.isModified) {
            continue;
        }

        QJsonObject obj;
        obj["partName"] = partName;

        QJsonArray electrodesArray;
        for (const ElectrodeData& electrode : partData.electrodes) {
            QJsonObject electrodeObj;
            electrodeObj["electrodeName"] = electrode.electrodeName;
            electrodeObj["processPosition"] = electrode.processPosition;
            electrodeObj["dischargeParameter"] = electrode.dischargeParameter;
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
    clearElectrodeTable();

    if (m_currentPartName.isEmpty()) {
        return;
    }

    if (!m_partDataMap.contains(m_currentPartName)) {
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
    }

    m_addElectrodeBtn->setEnabled(true);
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

void PartElectrodeConfigDialog::checkUnsavedChanges()
{
    if (!m_hasUnsavedChanges) {
        close();
        return;
    }

    QMessageBox::StandardButton result = QMessageBox::question(this, "确认",
        "配置已修改，是否保存？",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch (result) {
    case QMessageBox::Save:
        if (validateData()) {
            saveConfigToFiles();
            close();
        }
        break;
    case QMessageBox::Discard:
        close();
        break;
    case QMessageBox::Cancel:
        break;
    }
}

void PartElectrodeConfigDialog::closeEvent(QCloseEvent* event)
{
    checkUnsavedChanges();
    event->ignore();
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
        m_partList->setCurrentRow(currentRow);
    }

    m_hasUnsavedChanges = true;
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

        if (!electrode.positionModified) {
            electrode.processPosition = newValue + "_ROW1";
            QTableWidgetItem* posItem = m_electrodeTable->item(row, COL_POSITION);
            if (posItem) {
                posItem->setText(electrode.processPosition);
            }
        }

        if (!electrode.parameterModified) {
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
    }

    partData.isModified = true;
    m_hasUnsavedChanges = true;
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
    accept();
}

void PartElectrodeConfigDialog::onCancel()
{
    checkUnsavedChanges();
}
