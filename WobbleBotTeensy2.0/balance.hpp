#pragma once
#include <vector>
#include "Filter.hpp"
#include "Wire.h"
#include "Adafruit_MCP4725.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#include "EEPROM.h"

#define EN 23
#define PH 22
#define ISEN 16
#define VSEN 17

#define BNO055_SAMPLERATE_DELAY_MS (100)


using std::vector;

const double A = 4004.1248783224964;
const double a2 = 0.003933040727480468; // = accel/tilt
const double B = -16.513773666411964;
namespace MoBo {
class System {
  public:
    System();
    void begin();
    int waitBNO();
    int waitENC();
    vector<double> readSensors();
    double controller(const vector<double>& ref);
    void driveMotor(int in, double speed);
    void setGains(vector<double> gainz);
    const vector<double>& updateEnc(void);
  private:
    unsigned long period;
    unsigned long prevTime;
    unsigned long prevTimeENC;
    unsigned long curTime;
    double prevCounts;
    double gyro;
    vector<double> state;
    vector<double> gains;
    Adafruit_MCP4725 dac;
    Adafruit_BNO055 bno;
    Encoder Garry;
    void setup_pins();
    void initialize_pins();
    void setup_orientation_sensor();
    void displaySensorDetails(void);
    void displaySensorStatus(void);
    void displayCalStatus(void);
};

}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
