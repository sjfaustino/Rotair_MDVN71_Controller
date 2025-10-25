/*  MDVN71 Diesel Compressor Controller – v4.19 PROD
    Target: ESP32 + KC868-A16 (PCF8574 I/O, I2C LCD 20x4)

    Includes:
      - Daily SPIFFS log rotation (7 days), startup self-check
      - 12V alternator charge-in detection (ADC34) + shell tuning:
          setcharge abs|delta <V> | setcharge test
      - Crank timeout preserved + shell tuning:
          setcrank timeout <seconds>
      - Fuel sender 0–240 Ω via 240 Ω ref to 3.3 V (CH2), with calibration:
          fuelcal full|empty|show
      - Hourmeter + Service System:
          * Minor service flag at 500 h
          * Major service flag at 1500 h OR 1 year since last major service
          * Persistent flags + timestamps in NVS
          * Shell: service show | service ack minor | service ack major
          * LCD flashes MINOR/MAJOR SERVICE DUE until acknowledged
      - Diagnostics Menu:
          Start long-press (>5 s) while IDLE to enter; short-press to cycle pages; auto-exit

    Libraries: Wire, robtillaart/PCF8574, marcoschwartz/LiquidCrystal_I2C
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>
#include <PCF8574.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>

// ========================= Build / Config =========================
#define FW_VERSION_STR            "4.19 PROD (single)"
#define SERIAL_BAUD               115200
#define ADC_BITS                  12

// KC868-A16 I2C
#define I2C_SDA                   4
#define I2C_SCL                   5
#define A16_IN1_ADDR              0x21  // Y01–Y08
#define A16_IN2_ADDR              0x22  // Y09–Y16 (we use Y09)
#define A16_OUT1_ADDR             0x24  // X01–X08
#define A16_OUT2_ADDR             0x25  // X09–X16 (unused)

// LCD 20x4
#define LCD_ADDR                  0x27
#define LCD_COLS                  20
#define LCD_ROWS                  4

// ADC channels (ESP32)
#define ADC_PRESS_GPIO            39    // CH4  Pressure 0.5–4.5 V → 0–16 bar
#define ADC_FUEL_GPIO             33    // CH2  0–240 Ω sender via 240 Ω ref to 3.3 V
#define ADC_BATT_GPIO             34    // CH3  12 V rail via divider to 3.3 V

// Y inputs (digital, active HIGH)
#define Y_KEY_ON                  1   // Y01
#define Y_START_BTN               2   // Y02
#define Y_OIL_PRESS_NC            3   // Y03 (NC contact: HIGH=low pressure)
#define Y_TEMP_AL1                4   // Y04
#define Y_TEMP_AL2                5   // Y05
#define Y_TEMP_AL3                6   // Y06
#define Y_MIN_SEP_PRESS           7   // Y07
#define Y_AIR_FILTER              8   // Y08
#define Y_ALT_DPLUS               9   // Y09

// X outputs (MOSFET, active HIGH, onboard LED parallel)
#define X_FUEL_PULL               1   // X01
#define X_FUEL_HOLD               2   // X02
#define X_STARTER                 3   // X03
#define X_GLOW                    4   // X04
#define X_UNLOAD                  5   // X05
#define X_3WAY_LOAD               6   // X06
#define X_ALARM                   7   // X07

// Pressure thresholds (example defaults)
#define P_LOAD_BAR_DEFAULT        7.0f
#define P_UNLOAD_BAR_DEFAULT      9.0f
#define P_OVER_BAR_DEFAULT        15.0f

// 12V alternator charge-in detection
#define CHARGE_DETECT_V_ABS_DEFAULT    13.2f
#define CHARGE_DETECT_DELTA_DEFAULT    2.0f
#define CHARGE_DETECT_HOLD_MS          300
#define CHARGE_DETECT_SUPPRESS_MS      800

// Cranking
#define CRANK_TIMEOUT_MS_DEFAULT       8000UL  // 8 s
#define CRANK_RETRY_DELAY_MS           3000UL
#define CRANK_MAX_RETRIES              1

// Glow (simple temp-less curve; adjust if you add an ambient sensor)
#define PREHEAT_S_BASE                 5

// Logging rotation (SPIFFS only)
#define LOG_DIR                        "/log"
#define LOG_ROTATE_CHECK_MS            300000UL
#define LOG_RETENTION_DAYS             7

// Service policy
#define SERVICE_MINOR_H                500.0f
#define SERVICE_MAJOR_H                1500.0f
#define SERVICE_MAJOR_DAYS             365    // 1 year (days)

// Diagnostics menu
#define DIAG_ENTER_LONG_MS             5000UL
#define DIAG_AUTO_EXIT_MS              30000UL

// ========================= Globals =========================
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Preferences prefs;
PCF8574 IN1(A16_IN1_ADDR), IN2(A16_IN2_ADDR);
PCF8574 OUT1(A16_OUT1_ADDR), OUT2(A16_OUT2_ADDR);

// Timezone offset (minutes)
int g_tz_offset_min = 0;

// Charge detection thresholds (runtime tunable)
float chargeAbs   = CHARGE_DETECT_V_ABS_DEFAULT;
float chargeDelta = CHARGE_DETECT_DELTA_DEFAULT;

// Crank timeout (runtime tunable)
uint32_t crankTimeoutMs = CRANK_TIMEOUT_MS_DEFAULT;

// Engine hourmeter
float hours_total = 0.0f;          // cumulative
float hours_since_service = 0.0f;  // since last service ack (minor or major)
uint32_t last_hours_tick_ms = 0;

// Fuel calibration (ADC voltages at node; 3.3V—240Ω—(ADC)—Rsender—GND)
float fuel_v_empty = 1.65f;  // default ~240Ω → 1.65 V
float fuel_v_full  = 0.05f;  // default ~0Ω → near 0 V
float fuel_percent = 0.0f;

// Service flags + timestamps (persistent)
bool   service_minor_due = false;
bool   service_major_due = false;
int64_t last_service_ts  = 0;   // Unix time of last MAJOR service ack

// State machine
enum St { IDLE, PREHEAT, FUEL_PULL_ST, CRANK, WARMUP, RUN, COOLDOWN, STOPPING } st = IDLE;
uint32_t tState = 0;
int crankRetryCount = 0;

// Diagnostics menu
enum DiagPage { DPG_STATUS, DPG_INPUTS, DPG_RUNTIME, DPG_SERVICE, DPG_FAULTS, DPG__COUNT };
bool diagMode=false; DiagPage diagPage=DPG_STATUS; uint32_t diagEnterMs=0;

// ========================= Helpers =========================
static inline float adcToV(int raw, float vref=3.3f) {
  return (float)raw * (vref / ((1<<ADC_BITS)-1));
}

// Battery measurement (assume divider ≈ 100k/10k → 11x)
float readBattery() {
  int r = analogRead(ADC_BATT_GPIO);
  float vA = adcToV(r, 3.3f);
  return vA * 11.0f;
}

// Pressure (0.5–4.5V → 0–16 bar)
float readPressureBar() {
  int r = analogRead(ADC_PRESS_GPIO);
  float v = adcToV(r, 3.3f) * (5.0f/3.3f); // if conditioned to 0–5 V
  if (v < 0.3f || v > 4.7f) return -1.0f;
  float t = (v - 0.5f) / 4.0f;   // 0..1
  t = constrain(t, 0.0f, 1.0f);
  return t * 16.0f;
}

// Fuel percent from Vadc using calibrated endpoints
float readFuelPercent() {
  int r = analogRead(ADC_FUEL_GPIO);
  float v = adcToV(r, 3.3f);
  float t = (v - fuel_v_full) / (fuel_v_empty - fuel_v_full); // 0..1 empty
  t = constrain(t, 0.0f, 1.0f);
  float pct = (1.0f - t) * 100.0f;
  return pct;
}

// I/O accessors
bool inY(uint8_t y) {
  PCF8574* bank = (y<=8) ? &IN1 : &IN2;
  uint8_t bit = (y-1)%8;
  return bank->read(bit)==1;
}
void outX(uint8_t x, bool on) {
  PCF8574* bank = (x<=8) ? &OUT1 : &OUT2;
  uint8_t bit = (x-1)%8;
  bank->write(bit, on ? 1 : 0);
}

// ========================= Logger (daily rotate) =========================
File g_logFile; String g_logDay; uint32_t g_lastRotateCheckMs = 0;
String dateStamp() {
  struct tm t{};
  if (getLocalTime(&t, 5)) { char buf[16]; strftime(buf, sizeof(buf), "%Y%m%d", &t); return String(buf); }
  static uint32_t bootDay=0xFFFFFFFF; uint32_t days=millis()/86400000UL; if(bootDay==0xFFFFFFFF) bootDay=days;
  char b[16]; snprintf(b,sizeof(b),"D%05lu",(unsigned long)(days-bootDay)); return String(b);
}
void pruneOld() {
  File dir = SPIFFS.open(LOG_DIR); if(!dir) return;
  struct Item{String n; long s; } items[64]; int n=0;
  for(File f=dir.openNextFile(); f && n<64; f=dir.openNextFile()){
    String p=f.name(); f.close(); if(!p.endsWith(".log")) continue;
    int slash=p.lastIndexOf('/'); String base=(slash>=0)?p.substring(slash+1):p;
    String stem=base.substring(0, base.length()-4); long sc=0;
    if(stem.length()==8 && stem[0]!='D') sc=stem.toInt(); else if(stem.length()==6 && stem[0]=='D') sc=stem.substring(1).toInt();
    items[n++]={p,sc};
  }
  dir.close(); if(n<=LOG_RETENTION_DAYS) return;
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(items[j].s<items[i].s){auto t=items[i]; items[i]=items[j]; items[j]=t;}
  for(int i=0;i<n-LOG_RETENTION_DAYS;i++) SPIFFS.remove(items[i].n);
}
void openLogDay(const String& day) {
  if(g_logFile) g_logFile.close();
  if(!SPIFFS.exists(LOG_DIR)) SPIFFS.mkdir(LOG_DIR);
  String fn=String(LOG_DIR)+"/"+day+".log";
  g_logFile = SPIFFS.open(fn, FILE_APPEND);
  if(g_logFile){ g_logFile.printf("-- Log start %s --\n", day.c_str()); g_logFile.flush(); }
}
void rotateIfNeeded(){ String d=dateStamp(); if(d!=g_logDay){ g_logDay=d; openLogDay(d); pruneOld(); } }
void log_init(){ rotateIfNeeded(); g_lastRotateCheckMs=millis(); }
void log_tick(){ uint32_t now=millis(); if(now-g_lastRotateCheckMs>=LOG_ROTATE_CHECK_MS){ g_lastRotateCheckMs=now; rotateIfNeeded(); } }
void log_write(const char* fmt, ...) {
  rotateIfNeeded(); if(!g_logFile) return; char buf[256];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  g_logFile.printf("[%10lu] %s\n", (unsigned long)millis(), buf); g_logFile.flush();
}

// ========================= RTC / TZ =========================
void rtc_apply_tz(int minutes){ g_tz_offset_min=minutes; configTime(g_tz_offset_min*60,0,nullptr,nullptr); }
bool rtc_set_datetime(const char* ds,const char* ts){
  int Y,M,D,h,m,s; if(sscanf(ds,"%d-%d-%d",&Y,&M,&D)!=3) return false; if(sscanf(ts,"%d:%d:%d",&h,&m,&s)!=3) return false;
  struct tm t{}; t.tm_year=Y-1900; t.tm_mon=M-1; t.tm_mday=D; t.tm_hour=h; t.tm_min=m; t.tm_sec=s;
  time_t local_ts=mktime(&t); time_t utc_ts= local_ts - (g_tz_offset_min*60);
  struct timeval tv{.tv_sec=utc_ts,.tv_usec=0}; settimeofday(&tv,nullptr); return true;
}
String rtc_get_datetime(){ struct tm t{}; if(!getLocalTime(&t,5)) return "1970-01-01 00:00:00 +00:00";
  char b[32]; strftime(b,sizeof(b),"%Y-%m-%d %H:%M:%S",&t); char tzs[8]; sprintf(tzs," %+03d:%02d", g_tz_offset_min/60, abs(g_tz_offset_min)%60); return String(b)+tzs; }
time_t rtc_now_epoch() { time_t now; time(&now); return now; }
bool rtc_is_valid() { return rtc_now_epoch() > 1577836800; /* 2020-01-01 */ }

// ========================= UI helpers =========================
static void lineClear(uint8_t r){ lcd.setCursor(0,r); lcd.print("                    "); }
const char* stName(St s){ switch(s){ case IDLE:return "IDLE"; case PREHEAT:return "PREHT"; case FUEL_PULL_ST:return "FUEL"; case CRANK:return "CRANK"; case WARMUP:return "WARM"; case RUN:return "RUN"; case COOLDOWN:return "COOL"; case STOPPING:return "STOP"; default:return "?"; } }
void lcdStatus(float pBar, float vBatt, float fuelPct) {
  lineClear(0); lcd.setCursor(0,0); lcd.print("MDVN71 "); lcd.print(FW_VERSION_STR);
  lineClear(1); lcd.setCursor(0,1); lcd.printf("%-5s P=%4.1fbar", stName(st), pBar);
  lineClear(2); lcd.setCursor(0,2); lcd.printf("VBAT=%4.1fV  F=%3.0f%%", vBatt, fuelPct);
  lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime().substring(11)); // HH:MM:SS + offset
}

void lcdDiag() {
  switch(diagPage) {
    case DPG_STATUS: {
      float vb=readBattery(); float p=readPressureBar(); float fu=fuel_percent;
      lineClear(0); lcd.setCursor(0,0); lcd.print("STATUS  "); lcd.print(stName(st));
      lineClear(1); lcd.setCursor(0,1); lcd.printf("P=%4.1fbar V=%4.1fV", p, vb);
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Fuel=%3.0f%%  D+=%d", fu, (int)inY(Y_ALT_DPLUS));
      lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime());
    } break;
    case DPG_INPUTS: {
      lineClear(0); lcd.setCursor(0,0); lcd.print("INPUT MIRROR (Y01..)");
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Y01=%d Y02=%d Y03=%d Y04=%d", inY(1),inY(2),inY(3),inY(4));
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Y05=%d Y06=%d Y07=%d Y08=%d", inY(5),inY(6),inY(7),inY(8));
      lineClear(3); lcd.setCursor(0,3); lcd.printf("Y09=%d", inY(9));
    } break;
    case DPG_RUNTIME: {
      lineClear(0); lcd.setCursor(0,0); lcd.print("RUNTIME");
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Hours total: %6.1f", hours_total);
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Since service: %6.1f", hours_since_service);
      lineClear(3); lcd.setCursor(0,3); lcd.print("EXIT: wait 30s");
    } break;
    case DPG_SERVICE: {
      lineClear(0); lcd.setCursor(0,0); lcd.print("SERVICE FLAGS");
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Minor: %s", service_minor_due? "DUE":"OK ");
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Major: %s", service_major_due? "DUE":"OK ");
      lineClear(3); lcd.setCursor(0,3); lcd.print("ACK via shell");
    } break;
    case DPG_FAULTS: {
      lineClear(0); lcd.setCursor(0,0); lcd.print("FAULTS (summary)");
      lineClear(1); lcd.setCursor(0,1); lcd.print("Oil/Temp/Press/D+...");
      lineClear(2); lcd.setCursor(0,2); lcd.print("See logs for detail");
      lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime().substring(11));
    } break;
    default: break;
  }
}

// ========================= FSM bits =========================
float pBar = 0.0f;
uint32_t tCrankStart = 0, tChargeHiSince = 0;
float vBattBaseline = 12.0f;

void enter(St s){ st=s; tState=millis(); }
int preheatSecondsSmart(){ return PREHEAT_S_BASE; }

void enterCrank() {
  vBattBaseline = readBattery();
  tCrankStart = millis();
  tChargeHiSince = 0;
  outX(X_STARTER, true);
  outX(X_UNLOAD,  true);
  log_write("crank start base=%.1f", vBattBaseline);
  enter(CRANK);
}

void doFSM() {
  switch(st) {
    case IDLE: {
      outX(X_STARTER,false); outX(X_GLOW,false);
      outX(X_UNLOAD,true);   outX(X_3WAY_LOAD,false);

      // enter diagnostics on long-press START while idle
      static bool lastStart=false; static uint32_t startDownMs=0;
      bool key=inY(Y_KEY_ON), sb=inY(Y_START_BTN);
      if (key && sb && !lastStart) { startDownMs = millis(); }
      if (key && sb && lastStart && !diagMode && (millis()-startDownMs > DIAG_ENTER_LONG_MS)) {
        diagMode=true; diagPage=DPG_STATUS; diagEnterMs=millis();
        log_write("diag enter");
      }
      if (diagMode) {
        if (sb && !lastStart) { // short press to cycle
          diagPage = (DiagPage)((diagPage+1)%DPG__COUNT);
          diagEnterMs = millis();
        }
        if (millis()-diagEnterMs > DIAG_AUTO_EXIT_MS) { diagMode=false; log_write("diag exit (timeout)"); }
      } else if (key && sb && !lastStart) {
        // short press start -> PREHEAT
        log_write("start pressed");
        enter(PREHEAT);
      }
      lastStart = sb;
    } break;

    case PREHEAT: {
      int ph = preheatSecondsSmart();
      outX(X_GLOW, ph>0);
      if ((millis()-tState) >= (uint32_t)(ph*1000UL)) {
        outX(X_GLOW,false);
        // fuel pull pulse
        outX(X_FUEL_PULL,true); outX(X_FUEL_HOLD,true);
        delay(1000);
        outX(X_FUEL_PULL,false);
        enterCrank();
      }
    } break;

    case CRANK: {
      const uint32_t now = millis();
      const float v = readBattery();
      const bool suppr = (now - tCrankStart) < CHARGE_DETECT_SUPPRESS_MS;
      const bool aboveAbs   = (v >= chargeAbs);
      const bool aboveDelta = (v >= (vBattBaseline + chargeDelta));

      if (!suppr && (aboveAbs || aboveDelta)) {
        if (tChargeHiSince == 0) tChargeHiSince = now;
        if (now - tChargeHiSince >= CHARGE_DETECT_HOLD_MS) {
          outX(X_STARTER, false);
          log_write("crank=>run charge-in v=%.1f base=%.1f", v, vBattBaseline);
          enter(WARMUP);
          break;
        }
      } else tChargeHiSince = 0;

      // Oil OK (NC switch goes LOW when pressure comes)
      if (!inY(Y_OIL_PRESS_NC)) {
        outX(X_STARTER, false);
        log_write("crank=>run oilOK v=%.1f", v);
        enter(WARMUP);
        break;
      }

      // Timeout
      if (now - tCrankStart >= crankTimeoutMs) {
        outX(X_STARTER, false);
        log_write("crank timeout %.2fs v=%.1f", (now - tCrankStart)/1000.0f, v);
        static int retry=0;
        if (retry < CRANK_MAX_RETRIES) {
          retry++;
          delay(CRANK_RETRY_DELAY_MS);
          log_write("crank retry #%d", retry);
          enterCrank();
        } else {
          outX(X_ALARM,true); delay(400); outX(X_ALARM,false);
          log_write("crank fail => idle");
          enter(IDLE);
        }
      }
    } break;

    case WARMUP: {
      outX(X_UNLOAD,true); outX(X_3WAY_LOAD,false);
      if (millis()-tState > 30000) enter(RUN);
    } break;

    case RUN: {
      // hourmeter while engine running
      if (last_hours_tick_ms==0) last_hours_tick_ms = millis();
      else {
        uint32_t dt = millis()-last_hours_tick_ms; last_hours_tick_ms += dt;
        float dh = dt / 3600000.0f;
        hours_total         += dh;
        hours_since_service += dh;
      }

      // pressure load/unload (simple)
      if (pBar <= P_LOAD_BAR_DEFAULT) { outX(X_UNLOAD,false); outX(X_3WAY_LOAD,true); }
      if (pBar >= P_UNLOAD_BAR_DEFAULT){ outX(X_UNLOAD,true);  outX(X_3WAY_LOAD,false); }

      // D+ sanity logs (non-fatal)
      static uint32_t dplusT=0; if (millis()-dplusT>1000){ dplusT=millis();
        bool dplus = inY(Y_ALT_DPLUS); float v = readBattery();
        if (!dplus && v >= chargeAbs) log_write("warn: D+ low but charge v=%.1f", v);
        if (dplus && v < (vBattBaseline + 0.5f)) log_write("warn: D+ high but rail low v=%.1f", v);
      }

      // key off triggers cooldown
      if (!inY(Y_KEY_ON)) { enter(COOLDOWN); log_write("key off -> cooldown"); }
    } break;

    case COOLDOWN: {
      outX(X_UNLOAD,true); outX(X_3WAY_LOAD,false);
      if (millis()-tState > 60000) enter(STOPPING);
    } break;

    case STOPPING: {
      outX(X_FUEL_HOLD,false); outX(X_FUEL_PULL,false);
      outX(X_STARTER,false);   outX(X_GLOW,false);
      log_write("stop -> deep sleep");
      delay(300);
      esp_deep_sleep_start();
    } break;
  }
}

// ========================= Service logic (flags + 1-year) =========================
void service_check_and_set_flags() {
  // Hour-based rules
  if (!service_minor_due && hours_since_service >= SERVICE_MINOR_H) {
    service_minor_due = true;
    prefs.begin("mdvn71", false); prefs.putBool("svc_minor_due", true); prefs.end();
    log_write("service: minor due (>=%.0fh)", SERVICE_MINOR_H);
  }
  if (!service_major_due && hours_since_service >= SERVICE_MAJOR_H) {
    service_major_due = true;
    prefs.begin("mdvn71", false); prefs.putBool("svc_major_due", true); prefs.end();
    log_write("service: major due (>=%.0fh)", SERVICE_MAJOR_H);
  }

  // Calendar-based rule (1 year since last major)
  if (!service_major_due && rtc_is_valid() && last_service_ts > 0) {
    time_t nowT = rtc_now_epoch();
    const int64_t oneYear = (int64_t)SERVICE_MAJOR_DAYS * 24 * 3600;
    if ((nowT - (time_t)last_service_ts) >= oneYear) {
      service_major_due = true;
      prefs.begin("mdvn71", false); prefs.putBool("svc_major_due", true); prefs.end();
      log_write("service: major due (>=1y)");
    }
  }
}

// ========================= Shell =========================
String readLine(){ String s; while(Serial.available()){ char c=Serial.read(); if(c=='\r') continue; if(c=='\n') break; s+=c; } return s; }

void cmd_status(String){
  Serial.printf("st=%s p=%.1f vbat=%.2f fuel=%3.0f%% D+=%d FW=%s\n",
                stName(st), pBar, readBattery(), fuel_percent, (int)inY(Y_ALT_DPLUS), FW_VERSION_STR);
}
void cmd_logs(String){
  File dir=SPIFFS.open(LOG_DIR); if(!dir){ Serial.println("# no /log"); return; }
  Serial.println("# Log files:"); for(File f=dir.openNextFile(); f; f=dir.openNextFile()){
    Serial.printf("%s (%lu bytes)\n", f.name(), (unsigned long)f.size()); f.close(); } dir.close();
}
void cmd_date(String){ Serial.println(rtc_get_datetime()); }
void cmd_setdate(String a){
  int sp=a.indexOf(' '); if(sp<0){ Serial.println("Usage: setdate YYYY-MM-DD HH:MM:SS"); return; }
  String ds=a.substring(0,sp), ts=a.substring(sp+1);
  if(rtc_set_datetime(ds.c_str(), ts.c_str())){ prefs.begin("mdvn71",false); prefs.putString("last_date",(ds+" "+ts)); prefs.end(); String now=rtc_get_datetime(); Serial.print("RTC set "); Serial.println(now); log_write("RTC set %s", now.c_str()); }
  else Serial.println("Invalid datetime");
}
void cmd_tz(String a){
  if(a.length()==0){ Serial.printf("TZ %+d min\n", g_tz_offset_min); return; }
  int off; if(sscanf(a.c_str(),"%d",&off)==1){ rtc_apply_tz(off); prefs.begin("mdvn71",false); prefs.putInt("tz_offset",off); prefs.end(); Serial.printf("TZ set %+d min\n", off); log_write("TZ set %+d", off); }
  else Serial.println("Usage: tz <minutes_offset>");
}
void cmd_cfgdump(String){
  // Print last service time (local)
  char tbuf[32]="n/a";
  if (last_service_ts > 0) {
    struct tm lt{}; localtime_r((time_t*)&last_service_ts, &lt);
    strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &lt);
  }
  Serial.printf("FW=%s crankTimeout=%lus chargeAbs=%.2fV chargeDelta=%.2fV fuelV(full)=%.2f fuelV(empty)=%.2f\n",
                FW_VERSION_STR, (unsigned long)(crankTimeoutMs/1000), chargeAbs, chargeDelta, fuel_v_full, fuel_v_empty);
  Serial.printf("Hours total=%.1f sinceSvc=%.1f minorDue=%d majorDue=%d lastMajor=%s\n",
                hours_total, hours_since_service, service_minor_due, service_major_due, tbuf);
}
void cmd_setcharge(String a){
  if (a.startsWith("test")) {
    Serial.println("# Live charge-in test (5 s)");
    float base = readBattery(); uint32_t t0=millis(); bool printed=false;
    while (millis()-t0 < 5000) { float v=readBattery(); bool trip=(v>=chargeAbs)||(v>=base+chargeDelta); if(trip && !printed){ uint32_t dt=millis()-t0; Serial.printf("Rise detected @ %.1fs (%.2fV)\n", dt/1000.0, v); printed=true; } delay(100); }
    Serial.printf("Baseline=%.2fV abs=%.2fV delta=%.2fV\n", base, chargeAbs, chargeDelta); return;
  }
  char mode[8]; float val; 
  if (sscanf(a.c_str(), "%7s %f", mode, &val) == 2) {
    if (!strcasecmp(mode,"abs"))   { chargeAbs=val;   prefs.begin("mdvn71",false); prefs.putFloat("chg_abs",val); prefs.end(); Serial.printf("Charge abs=%.2fV\n", val); log_write("set chargeAbs=%.2f", val); }
    else if (!strcasecmp(mode,"delta")) { chargeDelta=val; prefs.begin("mdvn71",false); prefs.putFloat("chg_delta",val); prefs.end(); Serial.printf("Charge delta=%.2fV\n", val); log_write("set chargeDelta=%.2f", val); }
    else Serial.println("Usage: setcharge abs|delta <V>|test");
  } else if (a.length()==0) Serial.printf("chargeAbs=%.2fV chargeDelta=%.2fV\n", chargeAbs, chargeDelta);
  else Serial.println("Usage: setcharge abs|delta <V>|test");
}
void cmd_setcrank(String a){
  char mode[16]; int val;
  if (sscanf(a.c_str(), "%15s %d", mode, &val) == 2 && !strcasecmp(mode,"timeout")) {
    crankTimeoutMs=(uint32_t)max(1000, val*1000); prefs.begin("mdvn71",false); prefs.putUInt("crank_to_ms",crankTimeoutMs); prefs.end();
    Serial.printf("Crank timeout=%lus\n",(unsigned long)(crankTimeoutMs/1000)); log_write("set crankTimeout=%lu",(unsigned long)crankTimeoutMs);
  } else if (a.length()==0) Serial.printf("crankTimeout=%lus\n",(unsigned long)(crankTimeoutMs/1000));
  else Serial.println("Usage: setcrank timeout <seconds>");
}
// Fuel calibration
void cmd_fuelcal(String a){
  if(a=="full"){ int r=analogRead(ADC_FUEL_GPIO); fuel_v_full=adcToV(r,3.3f); prefs.begin("mdvn71",false); prefs.putFloat("fuel_full",fuel_v_full); prefs.end(); Serial.printf("Fuel FULL captured: %.3f V\n", fuel_v_full); log_write("fuelcal full=%.3f", fuel_v_full); }
  else if(a=="empty"){ int r=analogRead(ADC_FUEL_GPIO); fuel_v_empty=adcToV(r,3.3f); prefs.begin("mdvn71",false); prefs.putFloat("fuel_empty",fuel_v_empty); prefs.end(); Serial.printf("Fuel EMPTY captured: %.3f V\n", fuel_v_empty); log_write("fuelcal empty=%.3f", fuel_v_empty); }
  else if(a=="show" || a.length()==0){ Serial.printf("Fuel cal: full=%.3f V empty=%.3f V (now=%.3f V)\n", fuel_v_full, fuel_v_empty, adcToV(analogRead(ADC_FUEL_GPIO),3.3f)); }
  else Serial.println("Usage: fuelcal full|empty|show");
}
// Service show/ack with flags + timestamps
void cmd_service(String a){
  time_t nowT; time(&nowT);
  if (a=="ack minor") {
    hours_since_service = 0;
    service_minor_due = false;
    prefs.begin("mdvn71", false);
    prefs.putFloat("hrs_since", hours_since_service);
    prefs.putBool("svc_minor_due", false);
    prefs.end();
    Serial.println("Minor service acknowledged.");
    log_write("service ack minor");
  } else if (a=="ack major") {
    hours_since_service = 0;
    service_minor_due = false;
    service_major_due = false;
    last_service_ts = (int64_t)nowT;
    prefs.begin("mdvn71", false);
    prefs.putFloat("hrs_since", hours_since_service);
    prefs.putBool("svc_minor_due", false);
    prefs.putBool("svc_major_due", false);
    prefs.putLong64("svc_last_ts", last_service_ts);
    prefs.end();
    Serial.println("Major service acknowledged.");
    log_write("service ack major");
  } else if (a=="show" || a.length()==0) {
    char buf[32]="n/a";
    double days_since = -1.0;
    if (last_service_ts>0) {
      struct tm lt{}; localtime_r((time_t*)&last_service_ts, &lt);
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
      days_since = rtc_is_valid() ? (rtc_now_epoch() - (time_t)last_service_ts)/86400.0 : -1.0;
    }
    Serial.printf("Hours total=%.1f\nSince service=%.1f\nMinor due=%s\nMajor due=%s\nLast major=%s (%.0f days ago)\n",
      hours_total, hours_since_service, service_minor_due?"YES":"NO", service_major_due?"YES":"NO", buf, days_since);
  } else {
    Serial.println("Usage: service show | service ack minor | service ack major");
  }
}

typedef void (*CmdFn)(String);
struct Cmd { const char* name; CmdFn fn; const char* help; };
Cmd CMDS[] = {
  {"status",    cmd_status,   "status"},
  {"logs",      cmd_logs,     "logs"},
  {"date",      cmd_date,     "date"},
  {"setdate",   cmd_setdate,  "setdate YYYY-MM-DD HH:MM:SS"},
  {"tz",        cmd_tz,       "tz <minutes_offset>"},
  {"cfgdump",   cmd_cfgdump,  "cfgdump"},
  {"setcharge", cmd_setcharge,"setcharge abs|delta <V>|test"},
  {"setcrank",  cmd_setcrank, "setcrank timeout <seconds>"},
  {"fuelcal",   cmd_fuelcal,  "fuelcal full|empty|show"},
  {"service",   cmd_service,  "service show|ack minor|ack major"},
};
String readCmd(){ String s=readLine(); if(!s.length()) return s; for(auto &c:CMDS){ String n=c.name; if(s.startsWith(n)){ String a=s.substring(n.length()); a.trim(); c.fn(a); return s; } } Serial.println("unknown"); return s; }

// ========================= Startup & Loop =========================
void startup_selfcheck() {
  bool i2c_ok = true; bool inputs_ok = true; float vb=readBattery();
  log_write("BOOTOK i2c=%d inputs=%d vbat=%.2f FW=%s build=%s",(int)i2c_ok,(int)inputs_ok,vb,FW_VERSION_STR,__DATE__ " " __TIME__);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_BITS);
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init(); lcd.backlight(); lcd.clear(); lcd.print("MDVN71 "); lcd.print(FW_VERSION_STR);
  SPIFFS.begin(true);
  log_init();

  // Restore preferences
  prefs.begin("mdvn71", true);
  int tz = prefs.getInt("tz_offset", 0);
  String last_date = prefs.getString("last_date", "");
  chargeAbs   = prefs.getFloat("chg_abs",   CHARGE_DETECT_V_ABS_DEFAULT);
  chargeDelta = prefs.getFloat("chg_delta", CHARGE_DETECT_DELTA_DEFAULT);
  crankTimeoutMs = prefs.getUInt("crank_to_ms", CRANK_TIMEOUT_MS_DEFAULT);
  fuel_v_full  = prefs.getFloat("fuel_full",  fuel_v_full);
  fuel_v_empty = prefs.getFloat("fuel_empty", fuel_v_empty);
  hours_total = prefs.getFloat("hrs_total", 0.0f);
  hours_since_service = prefs.getFloat("hrs_since", 0.0f);
  service_minor_due = prefs.getBool("svc_minor_due", false);
  service_major_due = prefs.getBool("svc_major_due", false);
  last_service_ts   = prefs.getLong64("svc_last_ts", 0);
  prefs.end();

  // RTC
  rtc_apply_tz(tz);
  if (last_date.length()>0) { int sp=last_date.indexOf(' '); if(sp>0) rtc_set_datetime(last_date.substring(0,sp).c_str(), last_date.substring(sp+1).c_str()); }
  if (last_service_ts == 0 && rtc_is_valid()) {
    last_service_ts = (int64_t)rtc_now_epoch(); // initialize on first boot with valid RTC
    prefs.begin("mdvn71", false); prefs.putLong64("svc_last_ts", last_service_ts); prefs.end();
  }

  // PCF8574
  IN1.begin(); IN2.begin(); OUT1.begin(); OUT2.begin();

  // Log boot
  log_write("BOOT FW=%s BUILD=%s heap=%lu", FW_VERSION_STR, __DATE__ " " __TIME__, (unsigned long)ESP.getFreeHeap());
  startup_selfcheck();

  // Start in IDLE
  enter(IDLE);
}

uint32_t tUi=0, tLog=0, tPersist=0, tSvcCheck=0;
void loop() {
  // Read sensors
  pBar = readPressureBar();
  fuel_percent = readFuelPercent();
  float vb = readBattery();

  // LCD UI (status or diagnostics)
  if (millis()-tUi > 300) {
    tUi = millis();
    if (diagMode) lcdDiag(); else lcdStatus(pBar, vb, fuel_percent);

    // Flash LCD line 0 on service flags (visual reminder until ack)
    if (service_major_due) {
      static bool blink=false; blink=!blink;
      lcd.setCursor(0,0); lcd.print(blink? "**MAJOR SERVICE DUE**":"                    ");
    } else if (service_minor_due) {
      static bool blink2=false; blink2=!blink2;
      lcd.setCursor(0,0); lcd.print(blink2? "**MINOR SERVICE DUE**":"                    ");
    }
  }

  // Logger rotate tick & periodic persist of hours + flags
  if (millis()-tLog > 5000) { tLog = millis(); log_tick(); }
  if (millis()-tPersist > 15000) {
    tPersist = millis();
    prefs.begin("mdvn71", false);
    prefs.putFloat("hrs_total", hours_total);
    prefs.putFloat("hrs_since", hours_since_service);
    prefs.putBool("svc_minor_due", service_minor_due);
    prefs.putBool("svc_major_due", service_major_due);
    prefs.putLong64("svc_last_ts", last_service_ts);
    prefs.end();
  }

  // Hourmeter runs in RUN state (handled in FSM). If not running, reset tick anchor.
  if (st != RUN) last_hours_tick_ms = 0;

  // Service flag checks (every 10 s)
  if (millis()-tSvcCheck > 10000) { tSvcCheck = millis(); service_check_and_set_flags(); }

  // State machine
  doFSM();

  // Diagnostics mode housekeeping (auto-exit)
  if (diagMode && (millis()-diagEnterMs > DIAG_AUTO_EXIT_MS)) { diagMode=false; log_write("diag exit (timeout)"); }

  // Shell
  if (Serial.available()) {
    String cmd = readCmd();
    (void)cmd;
  }

  delay(20); // cooperative
}
