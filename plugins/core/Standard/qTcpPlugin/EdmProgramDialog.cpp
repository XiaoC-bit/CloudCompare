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
	contentLayout->addWidget(calcButton);

	QLabel* resultLabel = new QLabel("计算结果（补偿量）");
	resultLabel->setFont(f);
	contentLayout->addWidget(resultLabel);

	QGridLayout* resultGrid = new QGridLayout();
	resultGrid->setHorizontalSpacing(16);
	resultGrid->setVerticalSpacing(8);

	auto addResultRow = [&](const QString& label, QLabel*& valueLabel) {
		QLabel* lbl = new QLabel(label);
		lbl->setStyleSheet("color: #444444;");
		valueLabel = new QLabel("--");
		valueLabel->setStyleSheet("color: #111111; font-weight: 600; font-family: monospace;");
		resultGrid->addWidget(lbl, resultGrid->rowCount(), 0);
		resultGrid->addWidget(valueLabel, resultGrid->rowCount() - 1, 1);
	};

	addResultRow("X:", m_calculatedXLabel);
	addResultRow("Y:", m_calculatedYLabel);
	addResultRow("Z:", m_calculatedZLabel);
	addResultRow("B:", m_calculatedBLabel);
	addResultRow("C:", m_calculatedCLabel);

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
	if (!m_pointCloudService) {
		return;
	}

	Eigen::Vector3d P_machine = m_pointCloudService->getBAxisCenter();
	Eigen::Vector3d Q_machine = m_pointCloudService->getCAxisCenter();

	Eigen::Matrix3d R = T_icp.block<3, 3>(0, 0);
	Eigen::Vector3d t = T_icp.block<3, 1>(0, 3);

	double C_deg = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
	
	double sinC = std::sin(C_deg * M_PI / 180.0);
	double cosC = std::cos(C_deg * M_PI / 180.0);
	double r20_prime = R(2, 0) * cosC + R(2, 1) * sinC;
	double r22_prime = R(2, 2);
	double B_deg = std::atan2(-r20_prime, r22_prime) * 180.0 / M_PI;

	double B = B_deg * M_PI / 180.0;
	double C = C_deg * M_PI / 180.0;

	double first_spin_vector = -1;
	double second_spin_vector = -1;

	Eigen::Matrix3d Rb = Eigen::AngleAxisd(first_spin_vector * B, Eigen::Vector3d::UnitY()).toRotationMatrix();
	Eigen::Matrix3d Rc = Eigen::AngleAxisd(second_spin_vector * C, Eigen::Vector3d::UnitZ()).toRotationMatrix();

	Eigen::Matrix4d T_C;
	T_C.setIdentity();
	T_C.block<3, 3>(0, 0) = Rc;
	T_C.block<3, 1>(0, 3) = Q_machine - Rc * Q_machine;

	Eigen::Matrix4d T_B;
	T_B.setIdentity();
	T_B.block<3, 3>(0, 0) = Rb;
	T_B.block<3, 1>(0, 3) = P_machine - Rb * P_machine;

	Eigen::Matrix4d M_real = T_B * T_C;

	Eigen::Matrix4d M_ideal;
	M_ideal.setIdentity();
	M_ideal.block<3, 3>(0, 0) = R;

	Eigen::Vector4d O(0, 0, 0, 1);
	Eigen::Vector4d O_real = M_real * O;

	Eigen::Vector3d delta_icp = t - O_real.head<3>();

	m_calculatedX = delta_icp.x();
	m_calculatedY = delta_icp.y();
	m_calculatedZ = delta_icp.z();
	m_calculatedB = B_deg;
	m_calculatedC = C_deg;

	m_calculatedXLabel->setText(QString::number(m_calculatedX, 'f', 6));
	m_calculatedYLabel->setText(QString::number(m_calculatedY, 'f', 6));
	m_calculatedZLabel->setText(QString::number(m_calculatedZ, 'f', 6));
	m_calculatedBLabel->setText(QString::number(m_calculatedB, 'f', 3));
	m_calculatedCLabel->setText(QString::number(m_calculatedC, 'f', 3));
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
	if (m_calculatedX == 0 && m_calculatedY == 0 && m_calculatedZ == 0 && 
		m_calculatedB == 0 && m_calculatedC == 0) {
		Eigen::Matrix4d matrix;
		if (!parseMatrix(matrix)) {
			return false;
		}
		calculateCompensation(matrix);
	}

	return m_pointCloudService->executeEdmProgram(
		m_calculatedX, m_calculatedY, m_calculatedZ, 
		m_calculatedB, m_calculatedC);
}

void EdmProgramDialog::onOperationCompleted(bool success)
{
	if (success) {
		setProgressText("放电程序执行完成");
	} else {
		setProgressText("放电程序执行失败");
	}
}
