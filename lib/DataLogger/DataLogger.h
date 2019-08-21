#pragma once
#include <SPI.h>
#include <Fat16.h>
#include <Timer.h>

#define SUCCESS 0
#define ERROR 1
#define WAIT 2
//#define STR_LOG

struct Record
{
    struct comps_float
    {
        float x;
        float y;
        float z;
    };
    struct comps_int16
    {
        int x;
        int y;
        int z;
    };
    comps_float gyro;
    comps_int16 accelerometer;
    float altimeter;
    float pid_x;
    float pid_y;
    unsigned long time;
};

class DataLogger
{
private:
    SdCard sd;
    Fat16 file;       //binary file for data
    Fat16 error_file; //text file for error
    Timer timer;
    Record rec;
    // const uint8_t CS = 4;
    // const uint8_t CHIP_SELECT = SS;
    // uint8_t MISO = 12;
    // uint8_t MOSI = 11;
    // uint8_t CLK = 13;
    Gyroscope *gyro = nullptr;
    Altimeter *altimeter = nullptr;
    void collectData(const double pid_x, const double pid_y, const long time);

#ifdef STR_LOG
    template <typename T>
    inline String value(const T &val)
    {
        return val;
    }
    template <typename T, typename... Others>
    inline String value(const T &val, Others... rest) { return val + value(rest...); }
#endif
public:
    DataLogger(Gyroscope *gyro, Altimeter *altimeter, uint16_t log_res = 50); //log_res in hertz
    uint8_t setup();
    uint8_t storeData(double pid_x, double pid_y, long time);
    uint8_t check();
    uint8_t logError(String message, const long time);

#ifdef STR_LOG //0 = good; 1=bad;
    template <class... T>
    uint8_t collectReport(T... args); // 0=ok; 1=bad; 2=waiting
#endif
    ~DataLogger();
};

DataLogger::DataLogger(Gyroscope *gyro, Altimeter *altimeter, uint16_t log_res)
{
    this->gyro = gyro;
    this->altimeter = altimeter;
    long t = 1000 / log_res;
    timer = Timer(1000, t);
}

DataLogger::~DataLogger()
{
}

uint8_t DataLogger::setup()
{
    if ((!sd.begin(SS)) || (!Fat16::init(&sd)))
        return ERROR;
    return SUCCESS;
}

#ifdef STR_LOG
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
        return SUCCESS;
    }
    return WAIT;
}
#endif

void DataLogger::collectData(const double pid_x, const double pid_y, const long time)
{
    rec.gyro.x = this->gyro->getEuler().x;
    rec.gyro.y = this->gyro->getEuler().y;
    rec.gyro.z = this->gyro->getEuler().z;
    rec.accelerometer.x = this->gyro->getAcceleration().x;
    rec.accelerometer.y = this->gyro->getAcceleration().y;
    rec.accelerometer.z = this->gyro->getAcceleration().z;
    rec.altimeter = this->altimeter->getRocketAltitude();
    rec.pid_x = (float)pid_x;
    rec.pid_y = (float)pid_y;
    rec.time = time;
}

uint8_t DataLogger::check()
{
    if (!file.open("log.dat", O_RDWR | O_CREAT | O_AT_END))
    {
        return ERROR;
    }
    struct test
    {
        int checkme = 1;
        long t = 1000;
    };
    test rec;
    file.write((const uint8_t *)&rec, sizeof(rec));
    if (file.writeError)
    {
        file.close();
        return ERROR;
    }
    if (!file.remove("log.dat"))
        return ERROR;
    return SUCCESS;
}

uint8_t DataLogger::storeData(double pid_x, double pid_y, long time)
{
    timer.execute_every([&] {
        collectData(pid_x, pid_y, time);
        // const auto report = value(args...);
        if (!file.open("log.dat", O_RDWR | O_CREAT | O_AT_END))
        {
            return ERROR;
        }
        file.write((const uint8_t *)&rec, sizeof(rec));

        if (file.writeError)
        {
            file.close();
            return ERROR;
        }
        file.close();
        return SUCCESS;
    });
    return WAIT;
}

uint8_t DataLogger::logError(String message, const long time)
{
    if (!file.open("error.log", O_RDWR | O_CREAT | O_AT_END))
    {
        return ERROR;
    }
    String error = message + " at " + time + "\n";
    char *buf;
    error.toCharArray(buf, error.length());
    file.write(buf);
    if (file.writeError)
    {
        file.close();
        return ERROR;
    }
    file.close();
    return SUCCESS;
}
