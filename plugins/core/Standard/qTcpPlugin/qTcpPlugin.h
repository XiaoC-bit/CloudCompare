#pragma once

#include <ccStdPluginInterface.h>

class CcTcpServer;
class CommandDispatcher;
class PointCloudService;
class MachineProxy;
class LogDockWidget;
class QAction;

class qTcpPlugin : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qTcpPlugin" FILE "info.json")

public:
    explicit qTcpPlugin(QObject *parent = nullptr);
    ~qTcpPlugin() override;

	void            onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction*> getActions() override;
	void            setMainAppInterface(ccMainAppInterface* app) override; // 自动启动入口

private slots:
	void startServer();
	void stopServer();
	void showCalibrationDialog();
	void showProbeCalibrationDialog();
	void showRingCalibrationDialog();
	void showPartInspectDialog();
	void showElectrodeInspectDialog();
	void showPartElectrodeConfigDialog();
	void showAcquirePcdDialog();
	void showCalibrationResultDialog();
	void showEdmProgramDialog();
	void showSparkMachineProgramDialog();
	void toggleLogDock();
	void onLogDockVisibilityChanged(bool visible);

  private:
	QThread* m_tcpThread = nullptr;
  private:
	void               updateActions();
	QAction*           m_startAction;
	QAction*           m_stopAction;
	QAction*           m_calibrationAction;
	QAction*           m_probeCalibrationAction;
	QAction*           m_ringCalibrationAction;
	QAction*           m_partInspectAction;
	QAction*           m_electrodeInspectAction;
	QAction*           m_partElectrodeConfigAction;
	QAction*           m_acquirePcdAction;
	QAction*           m_calibrationResultAction;
	QAction*           m_edmProgramAction;
	QAction*           m_sparkMachineProgramAction;
	QAction*           m_toggleLogDockAction;
private:
    CcTcpServer*       m_server;
    CommandDispatcher* m_dispatcher;
    PointCloudService* m_pointCloudService;
    MachineProxy*      m_machineProxy;
    LogDockWidget*     m_logDock = nullptr;

    void initializeLogger();
};
