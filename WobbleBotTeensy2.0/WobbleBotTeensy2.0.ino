#include "balance.hpp"
#include "Filter.hpp"
MoBo::System sys;

const vector<double> gains = { 4224.25, 655.56,    0.0, -210.36,   0.0,   0.0,     0.0};
void setup() {
  sys.begin();
  //  sys.setGains({2000.0, 300000.0, 0, 0});
  //                tilt, gyro, counts, spd, accel, trans, trans_dot
  sys.setGains(gains);

  Serial.begin(115200);
  String header = "";
  header = "Time,Tilt,Gyro,counts,ctr";
  Serial.println(header);
  delay(1000);
}
int i = 0;
bool stp = true;

vector<double> ref = { 0.017, 0, 0, 0};
double ctr = 0.0;
vector<double> sens = vector<double>(4, 0.0);
void loop() {
  int cycles = sys.waitENC();
  sens = sys.updateEnc();
  sys.driveMotor(ctr, sens[4]);
  ref[0] = constrain(-0.05 + (sens[3] - sens[1]) * 0.03, -0.05 - 0.15, -0.05 + 0.15);
  //  ref[0] = -0.08;
  double accel_pred = a2 * sens[1] + B * ctr;
  if (sys.waitBNO()) {
    sens = sys.readSensors();

    for (int k = 0; k < 4; k++)Serial.print(String(sens[k], 5) + ",");
    Serial.print(ctr, 5);
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
        Serial.println();
  }
//  Serial.println();
  ctr = sys.controller(ref);
//  ctr=1;
}
