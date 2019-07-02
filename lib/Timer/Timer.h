#pragma once
#include <Arduino.h>

class Timer
{
    struct timer_data
    {
        long oldTime;
        // long delta = 0L;
        long current = 0L;
        long interval; //1 second
    };

private:
    timer_data everyTimer;
    timer_data forTimer;

public:
    Timer(long interval, int delta);
    Timer();
    ~Timer();
    bool execute_every(int delta);
    bool execute_every();
    bool execute_for();
    void setup();
};

Timer::Timer(long interval, int delta)
{
    this->forTimer.interval = interval;
    this->everyTimer.interval = delta;
}
Timer::Timer()
{
    this->forTimer.interval = 1000L; // 1 sec
    this->everyTimer.interval = 50L; // 20 Hz
}
Timer::~Timer()
{
}

void Timer::setup()
{
    forTimer.oldTime = millis();
    everyTimer.oldTime = millis();
}

bool Timer::execute_every(int delta) //wrap in if
{
    everyTimer.current = millis();
    if (everyTimer.current <= everyTimer.oldTime + delta)
    {
        return false;
    }
    everyTimer.oldTime = millis();
    return true;
}

bool Timer::execute_every() //wrap in if
{
    everyTimer.current = millis();
    if (everyTimer.current <= everyTimer.oldTime + this->everyTimer.interval)
    {
        return false;
    }
    everyTimer.oldTime = millis();
    return true;
}

bool Timer::execute_for() //wrap in while
{
    forTimer.current = millis();
    if (forTimer.current <= forTimer.oldTime + forTimer.interval)
    {
        return true;
    }
    forTimer.oldTime = millis();
    return false;
}
