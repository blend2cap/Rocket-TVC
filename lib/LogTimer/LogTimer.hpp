#pragma once
#include <Arduino.h>

class LogTimer
{
private:
    unsigned long T0;

public:
    LogTimer();
    ~LogTimer();
    void initLogTimer();
    unsigned long getLogTimer();
};

LogTimer::LogTimer(/* args */)
{
}

LogTimer::~LogTimer()
{
}

void LogTimer::initLogTimer()
{
    T0 = millis();
}
unsigned long LogTimer::getLogTimer()
{
    return millis() - T0;
}
