#pragma once

#include "SparkFunMPL3115A2.h"
#include "Timer.h"

class Altimeter
{
private:
    MPL3115A2 altimeter;
    Timer timer;

    float local_altitude;
    float rocket_altitude;
    void set_local_altitude();
    int samples = 0;

public:
    Altimeter();
    ~Altimeter();
    void setup();
    float getRocketAltitude();
    String log_altitude();
};

Altimeter::Altimeter()
{
}

Altimeter::~Altimeter()
{
}

void Altimeter::set_local_altitude()
{
    local_altitude = local_altitude + 1 / (samples + 1) * (altimeter.readAltitude() - local_altitude);
    samples++;
}

float Altimeter::getRocketAltitude()
{
    if (timer.execute_every(70))
        rocket_altitude = altimeter.readAltitude() - local_altitude;
    return rocket_altitude;
}

void Altimeter::setup()
{
    //Wire begin: communication already started by Gyroscope.h
    // altimeter.begin(); //get sensor online, but wire.begin already used in Gyroscope.h
    altimeter.setModeAltimeter();   // Measure altitude above sea level in meters
    altimeter.setOversampleRate(4); //read every 66 ms
    altimeter.enableEventFlags();
    timer.setup();
    //read for 1 second to set local altitude
    while (timer.execute_for())
    {
        if (timer.execute_every(70))
            set_local_altitude();
    }
}

String Altimeter::log_altitude()
{
    return (String)rocket_altitude;
}