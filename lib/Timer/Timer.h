#include <Arduino.h>

struct timer
{
    long oldTime;
    long delta = 0L;
    long current = 0L;
    long interval; //1 second
};

class Timer
{
private:
    timer everyTimer;
    timer forTimer;

public:
    Timer(long interval);
    Timer();
    ~Timer();
    bool execute_every(int delta);
    bool execute_for();
    void setup();
};

Timer::Timer(long interval)
{
    this->forTimer.interval = interval;
}
Timer::Timer()
{
    this->forTimer.interval = 1000L;
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
