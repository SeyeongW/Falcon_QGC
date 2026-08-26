#pragma once

#include "UnitTest.h"

class CompetitionDataControllerTest : public UnitTest
{
    Q_OBJECT

private slots:
    void _selectedArmDisarmRangeTest();
    void _lowRateRejectedTest();
};
