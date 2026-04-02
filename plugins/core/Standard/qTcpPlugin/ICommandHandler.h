#pragma once
#include "Command.h"

class ICommandHandler {
public:
    virtual ~ICommandHandler() = default;
    virtual void handle(const Command& cmd) = 0;
};
