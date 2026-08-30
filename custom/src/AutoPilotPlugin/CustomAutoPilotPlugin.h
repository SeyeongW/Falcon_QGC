#pragma once

#include "PX4AutoPilotPlugin.h"

class Vehicle;

class CustomAutoPilotPlugin : public PX4AutoPilotPlugin
{
    Q_OBJECT

public:
    explicit CustomAutoPilotPlugin(Vehicle *vehicle, QObject *parent = nullptr);
};
