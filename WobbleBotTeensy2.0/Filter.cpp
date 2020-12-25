#include "balance.hpp"
#include "Filter.hpp"
using namespace MoBo;

Filter::Filter(vector<double> TAPS) : idx(0), taps(TAPS) {
  NTAPS = taps.size();
  inBuf = vector<double>(NTAPS, 0.0);
  }

double Filter::filter(double in) {
  double out = 0;
  inBuf[idx] = in;
  double elem;
  double filt;
//  for(auto& elem : taps){
//    Serial.print(elem);
//    Serial.print(",");
//  }
  for (int i = 0; i < NTAPS; i++) {
//        Serial.println((NTAPS + idx - i) % NTAPS);
//Serial.print(String(taps[0])+",");
//        Serial.println(inBuf[idx]);
    elem = inBuf[((NTAPS + idx - i) % NTAPS)];
    filt = taps[i];
    out += filt*elem;
  }
//  Serial.println();
  idx = (idx + 1) % NTAPS;
  return out;
}
