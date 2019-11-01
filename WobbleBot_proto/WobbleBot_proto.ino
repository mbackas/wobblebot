#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <SPI.h>
//#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

//after much deliberation I have decided that the algorithm
//and code I had worked on last night at this time is nothing
// it interpolates samples (rather, it is supposed to), when
// There is already high resolution position data avaiable via
//the encoder itself. These values shoul be read "raw" from the
//wires.
// To interpret velocity however, we will need to perform
// measurements and calculations to extract the desired info
// v = dx/dt. we need one time difference, and one distance difference
//measure a step from the encoder, note the position, set time = 0
// measure another step, record the time elapsed. Calc. slope
//

# define nSLEEP 9
# define nRESET 5
# define I4 10
# define I3 11
# define I2 A2
# define I1 A1
# define I0 A0
# define ISEN A3
const int* current[5] = {A0, A1, A2, A3, A4};

# define EN 6
# define PH 7
# define DECAY 8

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const int chipSelect = 4;
//File myFile;
Encoder Rupert(2, 3);
long cur_counts = 0;
long prev_counts = 0;
long dif_counts = 0;

long ref = 0;
long error = 0;
long dut = 0;

unsigned long cur_time = 0;
unsigned long prev_time = 0;
unsigned long dif_time = 0;
unsigned long t0 = 0;

unsigned long* cur_state[4] = {0, 0, 0, 0};
unsigned long* prev_state[4] = {0, 0, 0, 0};

double slope = 0;
double prev_slope = 0;

unsigned int data_size = 255000;
String output = "";

volatile bool toggle1 = true;

const double factor = 2.0 * 3.1415926 / 197.304;
void setup_pins() {
  pinMode(nSLEEP, OUTPUT);
  pinMode(nRESET, OUTPUT);
  pinMode(I4, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I0, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(DECAY, OUTPUT);
}

void initialize_pins() {
  digitalWrite(nSLEEP, HIGH);
  digitalWrite(nRESET, HIGH);
  digitalWrite(I4, LOW);
  digitalWrite(I3, LOW);
  digitalWrite(I2, LOW);
  digitalWrite(I1, LOW);
  digitalWrite(I0, LOW);
  digitalWrite(PH, HIGH);
  digitalWrite(DECAY, LOW);
  digitalWrite(EN, LOW);
}

void setup_timer() {
  cli();//stop interrupts
  //set timer1 interrupt at 400Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  //TCCR1A=0b10100011;// fast pwm mode
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1khz increments
  OCR1A = 1999;// = (16*10^6) / (1000) - 1
  //had to use 16 bit timer1 for this bc 1999>255, but could switch to timers 0 or 2 with larger prescaler
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 bit for 1 prescaler
  TCCR1B |= (1 << CS01);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

void setup_orientation_sensor() {
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
     Look for the sensor's unique ID at the beginning oF EEPROM.
     This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }
  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (foundCalib) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    //    while (!bno.isFullyCalibrated())
    //    {
    //      bno.getEvent(&event);
    //      delay(BNO055_SAMPLERATE_DELAY_MS);
    //    }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
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

      /* Wait the specified delay before requesting new data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");
  ref = event.orientation.y;
  delay(500);
}

//void setup_SD(){
//  pinMode(chipSelect, OUTPUT);
//    // see if the card is present and can be initialized:
//    if (!SD.begin(chipSelect)) {
//      Serial.println("Card failed, or not present");
//      // don't do anything more:
//      //while (1) ;
//    }
//    //Serial.println("card initialized.");
//    SD.remove("motorpos.txt");
//    // Open up the file we're going to log to!
//    myFile = SD.open("motorpos.txt", O_CREAT | O_TRUNC | O_WRITE);
//    if (! myFile) {
//      //Serial.println("error opening datalog.txt");
//      // Wait forever since we cant write data
//      while (1) ;
//    }
//}




void setup() {
  setup_pins();
  initialize_pins();
  setup_timer();
  
  Serial.begin(115200);
  while (!Serial) {}
  
  // Orientation sensor setup //
  setup_orientation_sensor();

    // SD setup
  //setup_SD();
    
  delay(1000);
  prev_time = micros();
  t0 = micros();
  outputGPIO(dut);
  digitalWrite(EN, LOW);
}


int comp = 0;
double tilt = 0;
double tilt_dot = 0;
long eps = 3;
sensors_event_t event;

void loop() {
  digitalWrite(EN, HIGH);
  //j = number of ref changes
  for (int j = 0; j < 10000; j++) {
    //i = numbe of control evaluations before ref change
    for (int i = 0; i < 2000; i++) {
      bno.getEvent(&event);
      while(!toggle1){}
      prev_time = cur_time;
      cur_time = micros();
      
      tilt = event.orientation.y;
      tilt_dot = event.gyro.y;
      prev_counts = cur_counts;
      cur_counts = Rupert.read();
      /* Display the floating point data */
      display_data();
      /* Wait the specified delay before requesting new data */
      //delay(BNO055_SAMPLERATE_DELAY_MS);
      controller();
      toggle1 = false;
    }
  }
  digitalWrite(EN, LOW);
  while (1);
}

void controller(){
  //      dut = max(-31 + comp, min(31 - comp, 10.0 * error - 1.5 * slope));
      
      if(error>eps) dut = 10;
      else if(error<-eps) dut = -10;
      else dut = 0;
      
      if (slope > -1) {
        dut += comp;
      }
      else if (slope < 1) {
        dut -= comp;
      }
      if (dut < -0) {
        digitalWrite(PH, LOW);
      }
      else if (dut > 0) {
        digitalWrite(PH, HIGH);
      }
      //if (i % 10 == 0)Serial.println(slope);
      output = String(cur_time) + "," + String(cur_counts * factor) + "," + String(slope * factor, 4);
      outputGPIO(abs(dut));
}

void display_data(){
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.println(error);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");
}

ISR(TIMER1_COMPA_vect) { //timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (!toggle1) {
    digitalWrite(13, HIGH);
    toggle1 = 1;
  }
  else {
    digitalWrite(13, LOW);
    toggle1 = 1;
  }
}

//void read_SD(){
//  digitalWrite(EN, LOW);
//  //  Serial.println("Done");
//    myFile.close();
//    delay(1000);
//    myFile = SD.open("motorpos.txt");
//    if (myFile) {
//      // read from the file until there's nothing else in it:
//       while (myFile.available()) {
//         Serial.write(myFile.read());
//       }
//      // close the file:
//       myFile.close();
//       while(1);
//     } else {
//        // if the file didn't open, print an error:
//        Serial.println("error opening datalog.txt");
//     }
//}
void outputGPIO(unsigned int level) {
  unsigned int mask = 1;
  for (int k = 0; k < 5; k++) {
    if (mask & level) {
      digitalWrite(current[k], HIGH);
      //Serial.println("pin"+String(k)+" HIGH");
    }
    else {
      digitalWrite(current[k], LOW);
      //Serial.println("pin"+String(k)+" LOW");
    }
    mask = mask << 1;
  }
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
void displaySensorDetails(void)
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
  delay(500);
}
void displaySensorStatus(void)
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
  delay(500);
}
void displayCalStatus(void)
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
//void get_state(){
//  cur_state[1] = cur_state[0];
//  cur_state[0] = Rupert.read();
//}
