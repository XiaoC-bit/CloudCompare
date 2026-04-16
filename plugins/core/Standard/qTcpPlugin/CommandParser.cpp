#include "CommandParser.h"
#include <QJsonDocument>
#include <QJsonObject>

Command CommandParser::parse(const QString& json) {
    Command cmd;
    QJsonDocument doc = QJsonDocument::fromJson(json.toUtf8());
    if (doc.isObject()) {
        QJsonObject obj = doc.object();
		if (obj.contains("action"))
		{
			cmd.type = obj["action"].toString().toStdString();
		}
        if (obj.contains("Command")) {
            cmd.type = obj["Command"].toString().toStdString();
        }
        if (obj.contains("params")) {
            cmd.params = obj["params"].toObject();
        }
        if (obj.contains("IDCode")) {
            cmd.idCode = obj["IDCode"].toString();
        }
    }
    return cmd;
}
