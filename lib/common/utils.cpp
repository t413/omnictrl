#include <Arduino.h>
#include <rom/rtc.h>
#include "utils.h"


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float deadband(float v, float db) {
  //return 0 if v is within deadband
  //re-scale to have smooth transition to 0
  if (db == 0) return v;
  else if (v >  db) return (v - db);
  else if (v < -db) return (v + db);
  return 0;
}

float expo(float v, float e) {
  return copysign(pow(abs(v), e), v);
}

String str(const char *fmtStr, ...) {
  static char buf[201] = {'\0'};
  va_list arg_ptr;
  va_start(arg_ptr, fmtStr);
  vsnprintf(buf, 200, fmtStr, arg_ptr);
  va_end(arg_ptr);
  return String(buf);
}
String str(const std::string &s) {
  return String(s.c_str());
}
String str(bool v) {
  return v ? " WIN" : " FAIL";
}

StringPair split(const String &str, const String &del) {
  int at = str.indexOf(del);
  if (at >= 0) return StringPair(str.substring(0, at), str.substring(at + del.length()));
  return StringPair(str, "");
}

bool suffixed(String *str, const String &suff) {
  if (!str) return false;
  bool res = str->endsWith(suff);
  if (res) str->remove(str->lastIndexOf(suff));
  return res;
}

// float lifepo4_soc[] = {13.4, 13.3, 13.28, 13.};

String timeAgo(int sec) {
  int days = (sec / (3600 * 24));
  String ret = str("%ds", sec % 60);
  if (sec >= 60  ) ret = str("%dm ", (sec % 3600) / 60) + ret;
  if (sec >= 3600) ret = str("%dh ", ((sec % (3600 * 24)) / 3600)) + ret;
  if (days)        ret = str("%dd ", days % 365) + ret;
  if (days >= 365) ret = str("%dy ", days / 365) + ret;
  return ret;
}

String getResetReason(RESET_REASON r) {
  switch (r) {
    case NO_MEAN: return "";
    case POWERON_RESET         : return "Vbat power on reset";
    case DEEPSLEEP_RESET       : return "Deep Sleep reset digital core";
    case TG0WDT_SYS_RESET      : return "Timer Group0 Watch dog reset digital core";
    case TG1WDT_SYS_RESET      : return "Timer Group1 Watch dog reset digital core";
    case RTCWDT_SYS_RESET      : return "RTC Watch dog Reset digital core";
    case INTRUSION_RESET       : return "Instrusion tested to reset CPU";
    case RTCWDT_CPU_RESET      : return "RTC Watch dog Reset CPU";
    case RTCWDT_BROWN_OUT_RESET: return "Reset when the vdd voltage is not stable";
    case RTCWDT_RTC_RESET      : return "RTC Watch dog reset digital core and rtc module";
  }
  return "";
}

String getResetReasons() {
  return "Reset0: " + getResetReason(rtc_get_reset_reason(0)) + ". Reason1: " + getResetReason(rtc_get_reset_reason(1));
}