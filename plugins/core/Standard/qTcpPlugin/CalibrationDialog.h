#pragma once

#include "MachineStatusDialog.h"
#include <QVector3D>

class QTableWidget;
class QPushButton;
class QHBoxLayout;
class QLabel;
class QFrame;

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

	QVector<QVector3D> getPositions() const;

private slots:
	void onAddPosition();
	void onReset();
	void onDeleteRow();
	void onReadFromDevice();
	void onSavePositions();
	void onGeneratePositions();

protected:
	void setupAdditionalUI() override;
	void onOperationStarted() override;
	bool performOperation() override;
	void onOperationCompleted(bool success) override;
	void updateUIState() override;

private:
	void populateTable();
	static QVector<Position> loadDefaultPositions();
	static QVector<Position> defaultHardcodedPositions();

	QTableWidget* m_tableWidget;
	QPushButton* m_addButton;
	QPushButton* m_generateButton;
	QPushButton* m_resetButton;
	QPushButton* m_saveButton;
	QPushButton* m_startButton;
	QPushButton* m_cancelButton;
	QHBoxLayout* m_buttonLayout;
	QLabel* m_schematicLabel;
	QLabel* m_instructionLabel;
	QFrame* m_infoFrame;

	QVector<Position> m_positions;
	int m_defaultPositionCount;
	static const int MAX_POSITIONS;
};
