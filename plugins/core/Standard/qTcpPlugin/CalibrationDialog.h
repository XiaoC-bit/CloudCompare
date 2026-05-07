#pragma once

#include <QDialog>
#include <QVector3D>
#include <QProgressBar>
#include <QLabel>

class QVBoxLayout;
class QHBoxLayout;
class QPushButton;
class QTableWidget;
class PointCloudService;
class ccMainAppInterface;

class CalibrationDialog : public QDialog
{
private:
	friend class CalibrationGuard;
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
	explicit CalibrationDialog(ccMainAppInterface*app,PointCloudService* pointCloudService, QWidget* parent = nullptr);
    ~CalibrationDialog() override;

    static QVector<QVector3D> getDefaultPositions();
    QVector<QVector3D> getPositions() const;

private slots:
    void onAddPosition();
    void onStartCalibration();
    void onReset();
    void onDeleteRow();

private:
    void setupUI();
    void populateTable();

    QVBoxLayout *m_mainLayout;
    QTableWidget *m_tableWidget;
    QPushButton *m_addButton;
    QPushButton *m_resetButton;
    QPushButton *m_startButton;
    QPushButton *m_cancelButton;
    QHBoxLayout *m_buttonLayout;
    QProgressBar *m_progressBar;
    QLabel *m_progressLabel;

    PointCloudService *m_pointCloudService;
    QVector<Position> m_positions;
    static const QVector<Position> DEFAULT_POSITIONS;
    static const int MAX_POSITIONS = 30;

	ccMainAppInterface* m_app;

	void setCalibrationRunning(bool running);
	bool m_calibrationRunning = false;

  protected:
	void closeEvent(QCloseEvent* event) override;
};




// 在 .h 中或匿名命名空间里
class CalibrationGuard
{
  public:
	CalibrationDialog* dlg;
	explicit CalibrationGuard(CalibrationDialog* d)
	    : dlg(d)
	{
		dlg->setCalibrationRunning(true);
	}
	~CalibrationGuard()
	{
		dlg->setCalibrationRunning(false);
	}
};
