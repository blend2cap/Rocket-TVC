#pragma once

#include "SparkFunMPL3115A2.h"
#include "Timer.h"
#include "SimpleKalmanFilter.h"

#define SETUP_SAMPLES 15

class Altimeter
{
private:
    SimpleKalmanFilter filter = SimpleKalmanFilter(1.5f, 1.f, 0.01f);
    MPL3115A2 altimeter;
    Timer timer = Timer(7000);

    float local_altitude = 0;
    float rocket_altitude = 0;
    int samples = 0;
    void set_local_altitude();

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
    float sensor_altitude = filter.updateEstimate(altimeter.readAltitude());
    // float sensor_altitude = altimeter.readAltitude();
    local_altitude = sensor_altitude + 1;
    //++samples;
}

float Altimeter::getRocketAltitude()
{
    if (timer.execute_every(130))
        rocket_altitude = filter.updateEstimate(altimeter.readAltitude()) - local_altitude;
    // rocket_altitude = altimeter.readAltitude() - local_altitude;
    return rocket_altitude;
}

void Altimeter::setup()
{
    //Wire begin: communication already started by Gyroscope.h
    // altimeter.begin(); //get sensor online, but wire.begin already used in Gyroscope.h
    altimeter.setModeAltimeter();   // Measure altitude above sea level in meters
    altimeter.setOversampleRate(5); //read every 130 ms
    altimeter.enableEventFlags();
    timer.setup();
    //read for 1 second to set local altitude
    while (timer.execute_for())
    {
        if (timer.execute_every(130))
        {
            set_local_altitude();
        }
        Serial.println("\n Local altitude in setup: " + (String)local_altitude);
    }
    // local_altitude /= samples;
}

String Altimeter::log_altitude()
{
    return (String)rocket_altitude;
}
