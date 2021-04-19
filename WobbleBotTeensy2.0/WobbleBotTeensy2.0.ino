#include "balance.hpp"
#include "Filter.hpp"
MoBo::System sys;

//const vector<double> gains = { 4224.25, 655.56,    0.0, -210.36,   0.0,   0.0,     0.0};
const vector<double> gains = { 4224.25, 655.56, 186, 210.36,   0.0,   0.0,     0.0};


void setup() {
  sys.begin();
  sys.setGains(gains);

  Serial.begin(9600);
  String header = "Time,Tilt,Gyro,counts,ctr";
  //  Serial.println(header);
  delay(2000);
}
int i = 0;
bool stp = true;

vector<double> ref = { 0.017, 0, 0, 0};
double ctr = 0.0;
vector<double> sens = vector<double>(4, 0.0);
void loop() {
  int cycles = sys.waitENC();
  sens = sys.updateEnc();
  if (abs(sens[1]) > 0.4) {
    ctr = 0;
    sys.resetSensors();
    sys.driveMotor(ctr, sens[4]);
    while (abs(sens[1]) > 0.1) {
      sens = sys.readSensors();
    }
  }
  if(cycles<5) ctr=0;
  else ctr = sys.controller(ref);
  sys.driveMotor(ctr, sens[4]);
  //  ref[0] = constrain(-0.05 + (sens[3] - sens[1]) * 0.03, -0.05 - 0.15, -0.05 + 0.15);
  //  ref[0] = -0.08;
  //  double accel_pred = a2 * sens[1] + B * ctr;
  if (Serial.available()) {
    int idx = Serial.parseInt();
    double gn = Serial.parseFloat();
//    Serial.println("gain=" + String(gn));
//    Serial.println("idx=" + String(idx));
    while (Serial.available()) {
      Serial.read();
    }
    sys.changeIthGain(idx, gn);
  }
  if (sys.waitBNO()) {
    sens = sys.readSensors();
    //    printCSV();
  }
  //  Serial.println();
  //  ctr=1;
}

void printCSV() {
  String str = "";
  for (int k = 0; k < 5; k++)str += String(sens[k], 5) + ",";
  str += String(ctr);
  Serial.println(str);
  //    Serial.println("______sens_______");
  //    for (double& k : sens) {
  //      Serial.print(String(k) + ",");
  //    }
  //    Serial.print(",");
  //    Serial.print(ref[0], 5);
  //    Serial.print(",");
  //    Serial.print(sens[0], 5);
  //    Serial.print(",");
  //    Serial.print(gains[0] * (ref[0] - sens[1]), 5);
  //    Serial.print(",");
  //    Serial.print(-gains[1]*sens[2], 5);
  //    Serial.print(",");
  //    Serial.print(-gains[2]*sens[3], 5);
  //    Serial.print(",");
  //    Serial.print(-gains[3]*sens[4], 5);
  //    Serial.print(",");
  //    Serial.print(-gains[6]*sens[7], 5);
  //    Serial.print(sens[1], 5);
  //    Serial.print(",");
  //    Serial.print(1000.0 * sens[1]);
  //    Serial.print(",");
  //    Serial.print(analogRead(VSEN));
  //    Serial.print(",");
  //    Serial.print((1 - 2 * digitalRead(PH)) * 2.56 / 1024 * analogRead(ISEN));
  //  Serial.println();
}
