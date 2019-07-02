#pragma once
#include <SPI.h>
#include <Fat16.h>
#include <Timer.h>

#define ERROR 1
#define WAIT 2

class DataLogger
{
private:
    SdCard sd;
    Fat16 file;
    Timer timer;
    // const uint8_t CS = 4;
    // const uint8_t CHIP_SELECT = SS;
    // uint8_t MISO = 12;
    // uint8_t MOSI = 11;
    // uint8_t CLK = 13;

    template <typename T>
    inline String value(const T &val) { return val; }
    template <typename T, typename... Others>
    inline String value(const T &val, Others... rest) { return val + value(rest...); }

public:
    DataLogger(uint16_t log_res = 50); //log_res in hertz
    uint8_t setup();                   //0 = good; 1=bad;
    template <class... T>
    uint8_t collectReport(T... args); // 0=ok; 1=bad; 2=waiting
    ~DataLogger();
};

DataLogger::DataLogger(uint16_t log_res)
{
    long t = 1000 / log_res;
    timer = Timer(1000, t);
}

DataLogger::~DataLogger()
{
}

uint8_t DataLogger::setup()
{
    if (!sd.begin(SS))
        return 1;
    if (!Fat16::init(&sd))
        return 1;

    return 0;
}

template <class... T>
uint8_t DataLogger::collectReport(T... args)
{
    if (timer.execute_every())
    {
        const auto report = value(args...);
        if (!file.open("log.txt", O_RDWR | O_CREAT | O_AT_END))
        {
            return ERROR;
        }

        file.println(report);
        if (file.writeError)
            return ERROR;
        file.close();
        return 0;
    }
    return WAIT;
}