#include "EdmProgramDialog.h"
#include "PointCloudService.h"
#include <Eigen/Dense>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QClipboard>
#include <QApplication>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QDoubleSpinBox>
#include <ccMainAppInterface.h>

#include "CommLogger.h"

EdmProgramDialog::EdmProgramDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent)
	: MachineStatusDialog(app, pointCloudService, parent)
{
	setWindowTitle("生成放电程序");
	setMinimumWidth(500);
	init();
}

EdmProgramDialog::~EdmProgramDialog()
{
}

void EdmProgramDialog::setupAdditionalUI()
{
	QVBoxLayout* contentLayout = new QVBoxLayout();
	contentLayout->setSpacing(16);

	QLabel* matrixLabel = new QLabel("ICP配准矩阵（4x4）");
	QFont f = matrixLabel->font();
	f.setBold(true);
	f.setPointSize(f.pointSize() + 1);
	matrixLabel->setFont(f);
	contentLayout->addWidget(matrixLabel);

	m_matrixTable = new QTableWidget();
	m_matrixTable->setColumnCount(4);
	m_matrixTable->setRowCount(4);
	m_matrixTable->setFixedHeight(180);
	m_matrixTable->horizontalHeader()->setVisible(false);
	m_matrixTable->verticalHeader()->setVisible(false);
	m_matrixTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_matrixTable->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	m_matrixTable->setStyleSheet(R"(
		QTableWidget {
			border: 1px solid #d0d0d0;
			border-radius: 4px;
			font-family: "Consolas", "Courier New", monospace;
			font-size: 11px;
		}
		QTableWidget::item {
			padding: 2px 4px;
		}
	)");

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			QTableWidgetItem* item = new QTableWidgetItem(i == j ? "1.0" : "0.0");
			item->setTextAlignment(Qt::AlignCenter);
			m_matrixTable->setItem(i, j, item);
		}
	}
	contentLayout->addWidget(m_matrixTable);

	QHBoxLayout* buttonRow = new QHBoxLayout();
	buttonRow->setSpacing(10);

	QPushButton* pasteButton = new QPushButton("粘贴矩阵");
	pasteButton->setFixedHeight(32);
	pasteButton->setStyleSheet(R"(
		QPushButton {
			background-color: #607D8B;
			color: white;
			border: none;
			border-radius: 4px;
			font-weight: bold;
		}
		QPushButton:hover {
			background-color: #546E7A;
		}
	)");
	connect(pasteButton, &QPushButton::clicked, this, &EdmProgramDialog::onPasteMatrix);
	buttonRow->addWidget(pasteButton);

	QPushButton* calcButton = new QPushButton("计算补偿量");
	calcButton->setFixedHeight(32);
	calcButton->setStyleSheet(R"(
		QPushButton {
			background-color: #4CAF50;
			color: white;
			border: none;
			border-radius: 4px;
			font-weight: bold;
		}
		QPushButton:hover {
			background-color: #45a049;
		}
	)");
	connect(calcButton, &QPushButton::clicked, this, &EdmProgramDialog::onCalculateCompensation);
	buttonRow->addWidget(calcButton);

	contentLayout->addLayout(buttonRow);

	QLabel* resultLabel = new QLabel("补偿量（可编辑）");
	resultLabel->setFont(f);
	contentLayout->addWidget(resultLabel);

	QGridLayout* resultGrid = new QGridLayout();
	resultGrid->setHorizontalSpacing(12);
	resultGrid->setVerticalSpacing(8);

	auto addResultRow = [&](const QString& label, QDoubleSpinBox*& spinBox, double min, double max) {
		QLabel* lbl = new QLabel(label);
		lbl->setStyleSheet("color: #444444;");
		spinBox = new QDoubleSpinBox();
		spinBox->setRange(min, max);
		spinBox->setDecimals(6);
		spinBox->setFixedWidth(160);
		spinBox->setStyleSheet(R"(
			QDoubleSpinBox {
				border: 1px solid #d0d0d0;
				border-radius: 3px;
				padding: 4px;
			}
		)");
		resultGrid->addWidget(lbl, resultGrid->rowCount(), 0);
		resultGrid->addWidget(spinBox, resultGrid->rowCount() - 1, 1);
	};

	addResultRow("X:", m_xSpinBox, -9999.999, 9999.999);
	addResultRow("Y:", m_ySpinBox, -9999.999, 9999.999);
	addResultRow("Z:", m_zSpinBox, -9999.999, 9999.999);
	addResultRow("A:", m_aSpinBox, -180, 180);
	addResultRow("B:", m_bSpinBox, -180, 180);
	addResultRow("C:", m_cSpinBox, -180, 180);

	contentLayout->addLayout(resultGrid);

	m_mainLayout->addLayout(contentLayout);

	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->setSpacing(10);
	buttonLayout->addStretch();

	QPushButton* startButton = new QPushButton("确认执行");
	startButton->setFixedHeight(32);
	startButton->setFixedWidth(100);
	startButton->setStyleSheet(R"(
		QPushButton {
			background-color: #3366cc;
			color: white;
			border: none;
			border-radius: 4px;
			font-weight: bold;
		}
		QPushButton:hover {
			background-color: #2550a8;
		}
		QPushButton:disabled {
			background-color: #a0a0a0;
		}
	)");
	connect(startButton, &QPushButton::clicked, this, &EdmProgramDialog::onStartOperation);
	buttonLayout->addWidget(startButton);

	QPushButton* cancelButton = new QPushButton("取消");
	cancelButton->setFixedHeight(32);
	cancelButton->setFixedWidth(100);
	cancelButton->setStyleSheet(R"(
		QPushButton {
			background-color: #f0f0f0;
			color: #444444;
			border: 1px solid #d0d0d0;
			border-radius: 4px;
		}
		QPushButton:hover {
			background-color: #e0e0e0;
		}
	)");
	connect(cancelButton, &QPushButton::clicked, this, &EdmProgramDialog::reject);
	buttonLayout->addWidget(cancelButton);

	m_mainLayout->addLayout(buttonLayout);

	m_mainLayout->setSpacing(16);
	m_mainLayout->setContentsMargins(20, 20, 20, 20);
}

bool EdmProgramDialog::parseMatrix(Eigen::Matrix4d& matrix)
{
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			QTableWidgetItem* item = m_matrixTable->item(i, j);
			if (!item || item->text().isEmpty()) {
				return false;
			}
			bool ok = false;
			double val = item->text().toDouble(&ok);
			if (!ok) {
				return false;
			}
			matrix(i, j) = val;
		}
	}
	return true;
}

void EdmProgramDialog::calculateCompensation(const Eigen::Matrix4d& T_icp)
{
	if (!m_pointCloudService)
		return;

	// --- 机床参数 ---
	const Eigen::Vector3d P_machine = m_pointCloudService->getBAxisCenter();
	const Eigen::Vector3d Q_machine = m_pointCloudService->getCAxisCenter();

	// 工件坐标系原点（G54）在机床坐标系下的坐标
	// 工件坐标系相对机床坐标系只有纯平移，无旋转
	const Eigen::Vector3d P_workpiece = {-24.705, -93.46, -104.237};

	const double signB = 1.0;
	const double signC = 1.0;

	// --- Step 1: 将工件坐标系下的 T_icp 转换到机床坐标系 ---
	// T_he 退化为纯平移矩阵，R 部分为 I
	// T_machine = Trans(P_workpiece) * T_icp * Trans(-P_workpiece)
	// => R_machine = R_icp
	// => t_machine = t_icp + (I - R_icp) * P_workpiece
	const Eigen::Matrix3d R = T_icp.block<3, 3>(0, 0);
	const Eigen::Vector3d t = T_icp.block<3, 1>(0, 3)
	                          + (Eigen::Matrix3d::Identity() - R) * P_workpiece;

	// --- Step 2: ZY 分解，R = Rz(C)·Ry(B) ---
	// R(2,0) = -sinB*cosC
	// R(2,1) = -sinB*sinC
	// R(2,2) =  cosB
	// R(1,0) =  sinC*cosB
	// R(0,0) =  cosC*cosB
	const double C_rad = std::atan2(R(1, 0), R(0, 0));
	const double sinB  = -(R(2, 0) * std::cos(C_rad) + R(2, 1) * std::sin(C_rad));
	const double cosB  = R(2, 2);
	const double B_rad = std::atan2(sinB, cosB); // 有符号，范围(-π, π)
	const double B_deg = B_rad * 180.0 / M_PI;
	const double C_deg = C_rad * 180.0 / M_PI;

	// A角：残余旋转 R_res = Ry(-B)·Rz(-C)·R
	// CBC机床无法执行A轴补偿，仅显示参考；理想情况应≈0
	double           A_deg                 = 0.0;
	constexpr double GIMBAL_LOCK_THRESHOLD = 1e-2;
	if (std::abs(cosB) > GIMBAL_LOCK_THRESHOLD)
	{
		const Eigen::Matrix3d Rz_neg_C = Eigen::AngleAxisd(-C_rad,
		                                                   Eigen::Vector3d::UnitZ())
		                                     .toRotationMatrix();
		const Eigen::Matrix3d Ry_neg_B = Eigen::AngleAxisd(-B_rad,
		                                                   Eigen::Vector3d::UnitY())
		                                     .toRotationMatrix();
		const Eigen::Matrix3d R_res    = Ry_neg_B * Rz_neg_C * R;
		A_deg                          = std::atan2(R_res(1, 0), R_res(0, 0)) * 180.0 / M_PI;
	}
	else
	{
		// Gimbal lock：B≈±90°，C与A不可区分，约定A=0
		A_deg = 0.0;
	}

	// --- Step 3: Pivot-induced translation（BC转台，C轴随B轴转动）---
	const Eigen::Matrix3d Rb = Eigen::AngleAxisd(signB * B_rad,
	                                             Eigen::Vector3d::UnitY())
	                               .toRotationMatrix();
	const Eigen::Matrix3d Rc = Eigen::AngleAxisd(signC * C_rad,
	                                             Eigen::Vector3d::UnitZ())
	                               .toRotationMatrix();

	//// B轴固定，绕 P_machine 旋转
	//const Eigen::Vector3d t_pivotB = P_machine - Rb * P_machine;

	//// C轴中心随B轴转动，B转后C轴中心在机床基坐标系下的实际位置
	//const Eigen::Vector3d Q_actual = P_machine + Rb * (Q_machine - P_machine);

	//// C绕实际中心旋转产生的平移
	//const Eigen::Vector3d t_pivotC = Q_actual - Rc * Q_actual;

	//// 总 pivot 平移（Q_actual 已在机床基坐标系，直接叠加）
	//const Eigen::Vector3d t_pivot = t_pivotB + t_pivotC;


	

	const Eigen::Vector3d t_pivotB = P_machine - Rb * P_machine;
	const Eigen::Vector3d t_pivotC = Q_machine - Rc * Q_machine;
	// M_real * O 展开：
	const Eigen::Vector3d t_pivot = t_pivotB + Rb * t_pivotC;


	// --- Step 4: 真实平移补偿 = ICP平移 - pivot平移 ---
	const Eigen::Vector3d delta_xyz = t - t_pivot;

	// --- 输出到UI ---
	m_xSpinBox->setValue(delta_xyz.x());
	m_ySpinBox->setValue(delta_xyz.y());
	m_zSpinBox->setValue(delta_xyz.z());
	m_aSpinBox->setValue(A_deg); // 残余误差，仅参考
	m_bSpinBox->setValue(B_deg);
	m_cSpinBox->setValue(C_deg);
}

void EdmProgramDialog::onPasteMatrix()
{
	QClipboard* clipboard = QApplication::clipboard();
	QString text = clipboard->text();

	QJsonParseError parseError;
	QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &parseError);

	if (parseError.error != QJsonParseError::NoError || !doc.isArray()) {
		setProgressText("❌ 粘贴内容不是有效的JSON数组");
		return;
	}

	QJsonArray matrixArray = doc.array();
	if (matrixArray.size() != 4) {
		setProgressText("❌ 矩阵维度不正确（应为4x4）");
		return;
	}

	for (int i = 0; i < 4; ++i) {
		if (!matrixArray[i].isArray()) {
			setProgressText("❌ 矩阵格式不正确");
			return;
		}
		QJsonArray row = matrixArray[i].toArray();
		if (row.size() != 4) {
			setProgressText("❌ 矩阵维度不正确（应为4x4）");
			return;
		}
		for (int j = 0; j < 4; ++j) {
			if (!row[j].isDouble()) {
				setProgressText("❌ 矩阵元素必须为数字");
				return;
			}
			QTableWidgetItem* item = m_matrixTable->item(j, i);
			if (item) {
				item->setText(QString::number(row[j].toDouble(), 'f', 10));
			}
		}
	}

	setProgressText("✅ 矩阵粘贴成功");
}

void EdmProgramDialog::onCalculateCompensation()
{
	Eigen::Matrix4d matrix;
	if (!parseMatrix(matrix)) {
		setProgressText("❌ 矩阵解析失败，请检查输入格式");
		return;
	}

	calculateCompensation(matrix);
	setProgressText("✅ 补偿量计算完成");
}

void EdmProgramDialog::onOperationStarted()
{
	setProgressText("正在生成放电程序...");
}

bool EdmProgramDialog::performOperation()
{
	double x = -m_xSpinBox->value();
	double y = -m_ySpinBox->value();
	double z = -m_zSpinBox->value();
	double b = -m_bSpinBox->value();
	double c = -m_cSpinBox->value();

	return m_pointCloudService->executeEdmProgram(x, y, z, b, c);
}

void EdmProgramDialog::onOperationCompleted(bool success)
{
	if (success) {
		setProgressText("放电程序执行完成");
	} else {
		setProgressText("放电程序执行失败");
	}
}
