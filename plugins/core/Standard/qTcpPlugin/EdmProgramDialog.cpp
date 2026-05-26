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
	{
		return;
	}

	// --- 机床参数 ---
	const Eigen::Vector3d P_machine = m_pointCloudService->getBAxisCenter();   // B轴旋转中心（机床系）
	const Eigen::Vector3d Q_machine = m_pointCloudService->getCAxisCenter();   // C轴旋转中心（机床系）
	const Eigen::Matrix4d T_he      = m_pointCloudService->getHandEyeMatrix(); // scanner -> machine

	const double signB = -1.0; // B轴补偿方向符号，后续从配置读取
	const double signC = -1.0; // C轴补偿方向符号，后续从配置读取

	// --- Step 1: 将 T_icp 从扫描仪坐标系转换到机床坐标系 ---
	// T_icp 描述模型在扫描仪系下的偏差：P_scanner = T_icp * P_model
	// 机床系下等价变换：T_machine = T_he * T_icp * T_he_inv


	// --- Step 1: 取逆得到工件偏差，再转换到机床坐标系 ---
	// T_icp 是"点云往模型靠"的变换（偏差的逆），取逆得到实际工件偏差
	// 再通过 hand-eye 转换到机床坐标系：T_machine = T_he * T_dev * T_he_inv
	const Eigen::Matrix4d T_he_inv    = T_he.inverse();
	const Eigen::Matrix4d T_deviation = T_icp.inverse();
	const Eigen::Matrix4d T_machine   = T_he * T_deviation * T_he_inv;

	const Eigen::Matrix3d R = T_machine.block<3, 3>(0, 0);
	const Eigen::Vector3d t = T_machine.block<3, 1>(0, 3);

	// --- Step 2: ZY 分解（对应 CBC 机床运动链 Rz(C)·Ry(B)）---
	// R = Rz(C) · Ry(B)
	// R(2,0) = -sinB·cosC
	// R(2,1) = -sinB·sinC  =>  sinB = sqrt(R(2,0)^2 + R(2,1)^2)
	// R(2,2) =  cosB
	// R(1,0) = sinC·cosB,  R(0,0) = cosC·cosB  =>  C = atan2(R(1,0), R(0,0))（cosB≠0时）
	const double sinB_norm = std::sqrt(R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1));
	const double B_rad     = std::atan2(sinB_norm, R(2, 2));
	const double B_deg     = B_rad * 180.0 / M_PI;

	double C_deg = 0.0;
	double A_deg = 0.0; // 残余误差，CBC机床无法执行，仅显示参考

	constexpr double GIMBAL_LOCK_THRESHOLD = 1e-2;
	if (sinB_norm > GIMBAL_LOCK_THRESHOLD)
	{
		// 正常情况
		C_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;

		// A_deg：ZY分解后的残余旋转（绕Z'轴），即 R_residual = Ry(-B)·Rz(-C)·R
		// 对 CBC 机床，理想情况 A≈0；若偏大说明工件有绕刀轴的旋转误差
		const double cosB = std::cos(B_rad);
		const double sinC = std::sin(C_deg * M_PI / 180.0);
		const double cosC = std::cos(C_deg * M_PI / 180.0);
		// R_res = Ry(-B)·Rz(-C)·R，取其 atan2(R_res(1,0), R_res(0,0))
		// 展开后：
		const double r00r = R(0, 0) * cosC + R(1, 0) * sinC;
		const double r10r = -R(0, 0) * sinC + R(1, 0) * cosC;
		const double r01r = R(0, 1) * cosC + R(1, 1) * sinC;
		(void)r00r;
		(void)r01r; // 仅用r10r和对应项
		const double r11r = -R(0, 1) * sinC + R(1, 1) * cosC;
		A_deg             = std::atan2(r10r, r11r) * 180.0 / M_PI;
		// 注：r10r ≈ sinA·cosB，r11r ≈ cosA；当cosB≈1时退化准确
		(void)cosB;
	}
	else
	{
		// Gimbal lock：B≈0，C与A不可区分，约定C=0，全部归入A
		C_deg = 0.0;
		A_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
	}

	// --- Step 3: 计算 CBC 旋转矩阵及其 pivot-induced translation ---
	// 机床执行 signB*B 绕B轴枢轴旋转，再执行 signC*C 绕C轴枢轴旋转
	const Eigen::Matrix3d Rb = Eigen::AngleAxisd(signB * B_rad,
	                                             Eigen::Vector3d::UnitY())
	                               .toRotationMatrix();
	const Eigen::Matrix3d Rc = Eigen::AngleAxisd(signC * C_deg * M_PI / 180.0,
	                                             Eigen::Vector3d::UnitZ())
	                               .toRotationMatrix();

	// C轴先转（靠近工件侧），B轴后转；与 T_B * T_C 顺序一致
	const Eigen::Vector3d t_pivotC = Q_machine - Rc * Q_machine;
	const Eigen::Vector3d t_pivotB = P_machine - Rb * P_machine;

	// 枢轴引起的总平移（先C后B）
	// 工件经过C旋转后，B再旋转：B轴看到的C的平移也会被B带动
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
