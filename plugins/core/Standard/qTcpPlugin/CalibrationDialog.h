#pragma once

#include "MachineStatusDialog.h"
#include <QVector3D>

class QTableWidget;
class QPushButton;
class QHBoxLayout;

class CalibrationDialog : public MachineStatusDialog
{
	Q_OBJECT

private:
	struct Position
	{
		double x;
		double y;
		double z;

		Position(double x_, double y_, double z_)
		    : x(x_)
		    , y(y_)
		    , z(z_)
		{
		}
	};

public:
	explicit CalibrationDialog(ccMainAppInterface* app, PointCloudService* pointCloudService, QWidget* parent = nullptr);
	~CalibrationDialog() override;

	static QVector<QVector3D> getDefaultPositions();
	QVector<QVector3D> getPositions() const;

private slots:
	void onAddPosition();
	void onReset();
	void onDeleteRow();
	void onReadFromDevice();

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;

private:
	void populateTable();

	QTableWidget* m_tableWidget;
	QPushButton* m_addButton;
	QPushButton* m_resetButton;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;

	QVector<Position> m_positions;
	static const QVector<Position> DEFAULT_POSITIONS;
	static const int MAX_POSITIONS = 30;
};
