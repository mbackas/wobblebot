#include "balance.hpp"

using namespace MoBo;

using std::vector;


System::System(): period(10000), prevTime(0), Garry(Encoder(14, 15)) {}

void System::setGains(vector<double> gainz) {
  gains = gainz;
}

double System::controller(const vector<double>& ref) {
  double result = 0;
  for (int k = 1; k < state.size(); k++) {
    result += (ref[k - 1] - state[k]) * gains[k - 1];
  }
  return result;
}
void System::begin() {
  gains = {0, 0, 0, 0};
  state = vector<double>(8, 0.0);
  dac.begin(0x62);
  bno = Adafruit_BNO055(55, 0x28);
  setup_pins();
  initialize_pins();
  setup_orientation_sensor();
  pinMode(13, OUTPUT);
}

void System::setup_pins() {
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(21, INPUT);
}
void System::initialize_pins() {
  digitalWrite(PH, HIGH);
  digitalWrite(EN, LOW);
}
void System::setup_orientation_sensor() {
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check  your connections */
    Serial.print("Ooops, no BNO055 detected ... Check      your wiring or I2C ADDR!");
    while (1);
  }
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
     Look for the sensor's unique ID at the beginning oF   EEPROM.
     This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor  exists in EEPROM");
  }
  else
  {
    //    Serial.println("\nFound Calibration for this sensor    in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    //    displaySensorOffsets(calibrationData);

    //    Serial.println("\n\nRestoring Calibration data to the  BNO055...");
    bno.setSensorOffsets(calibrationData);
    //    Serial.println("\n\nCalibration data loaded into       BNO055");
    foundCalib = true;
  }

  /* Display some basic information on this sensor */
  //  displaySensorDetails();
  //
  //    /* Optional: Display current status */
  //  displaySensorStatus();

  /* Crystal must be configured AFTER loading calibration  data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration    very often */
  if (foundCalib) {
    //            Serial.println("Move sensor slightly to calibrate magnetometers");
    //                while (!bno.isFullyCalibrated())
    //                {
    //                  bno.getEvent(&event);
    //                  displayCalStatus();
    //                  Serial.println("");
    //                  delay(BNO055_SAMPLERATE_DELAY_MS);
    //                }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
      //      Serial.println(__LINE__);
      bno.getEvent(&event);
      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);

      /* Optional: Display calibration status */
      displayCalStatus();

      /* New line for the next sample */
      Serial.println("");

      /* Wait the specified delay before requesting new    data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  //  Serial.println("\nFully calibrated!");
  //  Serial.println("--------------------------------");
  //  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  //  displaySensorOffsets(newCalib);

  //  Serial.println("\n\nStoring calibration data to          EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  //  Serial.println("Data stored to EEPROM.");
  //
  //  Serial.println("\n--------------------------------\n");
}

int System::waitBNO() {
  curTime = micros();
  if (curTime - prevTime > period) {
    prevTime += period;
    return 1;
  }
  else {
    return 0;
  }
}



double prevGyro = 0.0;

vector<double> System::readSensors() {
  sensors_event_t event2;
  imu::Quaternion q = bno.getQuat();
  bno.getEvent(&event2, Adafruit_BNO055::VECTOR_GYROSCOPE);
  curTime = micros();
  gyro = event2.gyro.z*0.01745329251;
  double tilt = q.toEuler().y();

  //  Serial.print(gyro,5);
  //  Serial.print(",");
  //  Serial.print(gyroFilt,5);

  state[0] = curTime;
  state[1] = tilt;
  //
  //
  return state;
}

const int kin_fric_low = -1000;
const int kin_fric_high = 1000;
const int static_fric_high = 2000;
const int static_fric_low = -2000;


void System::driveMotor(int in, double spd) {
  int num = 0;
//  in=1;
  unsigned int dut = 256;
  bool phs = true;
  if (abs(spd) < 0.001) {
    dut = 256;
    if (in == 0) {
      num = 0;
    }
    else if (in > 0) {
      num = static_fric_high;
    }
    else if (in < 0) {
      num = static_fric_low;
    }
  }
  else if (in > 0.0) {
    num = kin_fric_high;
    dut = 256;
  }
  else if (in < 0.0) {
    num = kin_fric_low;
    dut = 256;
  }
  if(abs(state[4])>11.0)dut=150;
  num += in;
  //  num = in;
  if (num > 0)phs = true;
  else phs = false;
  digitalWrite(PH, phs);
  analogWrite(EN, dut);
  dac.setVoltage(constrain(abs(num), 0, 3000), false);
}



void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

void System::displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}
void System::displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
}

void System::displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
double prevCountsFilt = 0.0;

MoBo::Filter GFil(TAPS_GYRO);
MoBo::Filter CFil(TAPS_ENC);
MoBo::Filter TFil(TAPS_ENC);

const vector<double>& System::updateEnc(void) {
  curTime = micros();
  double cnts = Garry.read()*0.0106150666099;
  double countsFilt = CFil.filter(cnts);
  double spd = 1000000.0 * (countsFilt - prevCountsFilt) / (curTime - prevTimeENC);

  double gyroFilt = GFil.filter(gyro);
//  double tiltFilt = TFil.filter(state[1]);
//  double accel = 1000000.0 * (gyroFilt - prevGyro) / (curTime - prevTimeENC);
  prevGyro = gyroFilt;
  double trans = -0.000404434038 * countsFilt + 0.085725 * state[1];
  double trans_dot = -0.000404434038 * spd + 0.085725 * gyroFilt;

  state[0] = curTime;
//  state[1] = tilt;
  state[2] = gyro;
  state[3] = cnts;
  state[4] = spd;
//  state[5] = accel;
  state[6] = trans;
  state[7] = trans_dot;
  
  prevCounts = cnts;
  prevCountsFilt = countsFilt;
  return state;
}

int System::waitENC() {
  unsigned int count = 0;
  prevTimeENC += 1000;
  digitalWrite(13, HIGH);
  while (micros() - prevTimeENC < 1000) {
    count++;
  }
  digitalWrite(13, LOW);
  return count;
}
