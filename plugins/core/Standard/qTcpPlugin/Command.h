#pragma once
#include <string>
#include <QJsonObject>
#include <QTcpSocket>

struct Command {
    std::string type;
    QJsonObject params;
    QTcpSocket* socket;
    QString idCode;

	Command()
	{
		socket = nullptr;
	}
};
