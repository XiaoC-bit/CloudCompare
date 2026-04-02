#pragma once
#include <QString>
#include "Command.h"

class CommandParser {
public:
    static Command parse(const QString& json);
};
