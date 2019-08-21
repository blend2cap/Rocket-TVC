#pragma once
#include <Arduino.h>

class LogTimer
{
private:
    static unsigned long T0;

public:
    LogTimer();
    ~LogTimer();

    static void initLogT()
    {
        T0 = millis();
    }
    static const long getT()
    {
        return millis() - T0;
    }
};

unsigned long LogTimer::T0;

LogTimer::LogTimer(/* args */)
{
}

LogTimer::~LogTimer()
{
}
