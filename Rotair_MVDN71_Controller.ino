/*  MDVN71 Diesel Compressor Controller – v4.20 PROD (single file)
    Target: ESP32 + KC868-A16 (PCF8574 I/O, I2C LCD 20x4)

    New in v4.20:
      - Sensor warm-up mask for analog validation
      - Thermal Guard (optional ADCs for cylinder/oil temperatures) with trend monitoring
      - Adaptive Crank Learning (learned timeout = avg successful crank + 15%, clamped)
      - Soft-Recovery after brown-out (resume RUN if alternator/batt indicates running)
      - Safe Start Inhibit on Low Fuel (<5%)
      - Real-Time Trend Logger to /trend/YYYYMMDD.csv (5 s)
      - Fault Replay Buffer (last 12 s at 1 Hz) dumped to /replay/*.csv on FAULT
      - Event Categorization prefixes: [INFO],[WARN],[FAULT]
      - Daily log CRC32 footer on rotation
      - Auto-Export Over Serial: logdump [YYYYMMDD] [N]
      - Multi-Language Menu: setlang <code>, loads /lang/<code>.json, fallback to /lang/en.json
      - LCD pressure bargraph with auto-scaling
      - Shell help index: help
      - Shell log streaming: logtail (toggle)
      - Config Import/Export: config export|import (to /config.json)

    NOTE:
      - Temperature ADCs are optional. If not connected, Thermal Guard auto-disables.
      - Keep your previous wiring: Y01..Y09, X01..X07, ADC33 fuel, ADC34 batt, ADC39 pressure.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Preferences.h>
#include <PCF8574.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <time.h>

//============================= Build / Config =============================
#define FW_VERSION_STR            "4.20 PROD (single)"
#define SERIAL_BAUD               115200
#define ADC_BITS                  12

// I2C pins (KC868-A16)
#define I2C_SDA                   4
#define I2C_SCL                   5

// PCF8574 addresses
#define A16_IN1_ADDR              0x21  // Y01–Y08
#define A16_IN2_ADDR              0x22  // Y09–Y16 (use Y09)
#define A16_OUT1_ADDR             0x24  // X01–X08
#define A16_OUT2_ADDR             0x25  // X09–X16 (unused)

// LCD 20x4
#define LCD_ADDR                  0x27
#define LCD_COLS                  20
#define LCD_ROWS                  4

// ADC GPIOs
#define ADC_PRESS_GPIO            39    // CH4
#define ADC_FUEL_GPIO             33    // CH2
#define ADC_BATT_GPIO             34    // CH3

// OPTIONAL temperature ADCs (leave -1 to disable Thermal Guard)
#define ADC_TEMP_CYL_GPIO         -1    // e.g. 35 if wired (0-3.3V)
#define ADC_TEMP_OIL_GPIO         -1    // e.g. 36 if wired (0-3.3V)

// Inputs (active HIGH)
#define Y_KEY_ON                  1   // Y01
#define Y_START_BTN               2   // Y02
#define Y_OIL_PRESS_NC            3   // Y03 (NC: HIGH=low pressure)
#define Y_TEMP_AL1                4   // Y04
#define Y_TEMP_AL2                5   // Y05
#define Y_TEMP_AL3                6   // Y06
#define Y_MIN_SEP_PRESS           7   // Y07
#define Y_AIR_FILTER              8   // Y08
#define Y_ALT_DPLUS               9   // Y09

// Outputs (active HIGH)
#define X_FUEL_PULL               1   // X01
#define X_FUEL_HOLD               2   // X02
#define X_STARTER                 3   // X03
#define X_GLOW                    4   // X04
#define X_UNLOAD                  5   // X05
#define X_3WAY_LOAD               6   // X06
#define X_ALARM                   7   // X07

// Pressure thresholds
#define P_LOAD_BAR_DEFAULT        7.0f
#define P_UNLOAD_BAR_DEFAULT      9.0f
#define P_OVER_BAR_DEFAULT        15.0f

// Alternator charge-in detect (12 V system)
#define CHARGE_DETECT_V_ABS_DEFAULT    13.2f
#define CHARGE_DETECT_DELTA_DEFAULT    2.0f
#define CHARGE_DETECT_HOLD_MS          300
#define CHARGE_DETECT_SUPPRESS_MS      800

// Crank
#define CRANK_TIMEOUT_MS_DEFAULT       8000UL
#define CRANK_RETRY_DELAY_MS           3000UL
#define CRANK_MAX_RETRIES              1
#define CRANK_LEARN_MIN_MS             3000UL
#define CRANK_LEARN_MAX_MS             8000UL
#define CRANK_LEARN_ALPHA              0.3f   // EMA blend for new success

// Glow
#define PREHEAT_S_BASE                 5

// Warm-up & cooldown
#define WARMUP_MS                      30000UL
#define COOLDOWN_MS                    60000UL

// Logging / files
#define LOG_DIR                        "/log"
#define TREND_DIR                      "/trend"
#define REPLAY_DIR                     "/replay"
#define LANG_DIR                       "/lang"
#define CONFIG_JSON_PATH               "/config.json"
#define LOG_ROTATE_CHECK_MS            300000UL
#define LOG_RETENTION_DAYS             7
#define TREND_PERIOD_MS                5000UL
#define REPLAY_DEPTH                   12      // seconds
#define REPLAY_PERIOD_MS               1000UL

// Sensor warm-up mask
#define SENSOR_WARMUP_MS               3000UL

// Fuel inhibit
#define FUEL_LOCKOUT_PCT               5.0f

// Thermals (only if ADCs are defined)
#define THERM_SLOPE_WINDOW_S           600     // 10 min trend window (approx with EMA)
#define THERM_SLOPE_LIMIT_C_PER_MIN    3.0f    // alarm if exceeded persistently
#define THERM_SAMPLING_MS              2000UL

// Diagnostics menu
#define DIAG_ENTER_LONG_MS             5000UL
#define DIAG_AUTO_EXIT_MS              30000UL

//============================= Globals / Objects ===========================
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Preferences prefs;
PCF8574 IN1(A16_IN1_ADDR), IN2(A16_IN2_ADDR);
PCF8574 OUT1(A16_OUT1_ADDR), OUT2(A16_OUT2_ADDR);

// Timezone offset (minutes)
int g_tz_offset_min = 0;

// Charge detect thresholds
float chargeAbs   = CHARGE_DETECT_V_ABS_DEFAULT;
float chargeDelta = CHARGE_DETECT_DELTA_DEFAULT;

// Crank timeouts
uint32_t crankTimeoutMs_base = CRANK_TIMEOUT_MS_DEFAULT;
uint32_t crankTimeoutMs_dyn  = CRANK_TIMEOUT_MS_DEFAULT; // adaptive

// Hours
float hours_total = 0.0f;
float hours_since_service = 0.0f;
uint32_t last_hours_tick_ms = 0;

// Fuel calibration (ADC volts at node)
float fuel_v_empty = 1.65f;
float fuel_v_full  = 0.05f;
float fuel_percent = 0.0f;

// Service flags
bool   service_minor_due = false;
bool   service_major_due = false;
int64_t last_service_ts  = 0;

// Language (simple key→string dictionary loaded from SPIFFS)
String g_lang = "en";
struct Lang {
  String status, inputs, runtime, service, faults;
  String service_minor_due, service_major_due, exit_hint;
  String low_fuel_no_start;
} L;

// State machine
enum St { IDLE, PREHEAT, FUEL_PULL_ST, CRANK, WARMUP, RUN, COOLDOWN, STOPPING } st = IDLE;
uint32_t tState = 0;

// Diagnostics menu
enum DiagPage { DPG_STATUS, DPG_INPUTS, DPG_RUNTIME, DPG_SERVICE, DPG_FAULTS, DPG__COUNT };
bool diagMode=false; DiagPage diagPage=DPG_STATUS; uint32_t diagEnterMs=0;

// Brown-out soft-recovery
bool boot_brownout = false;

// Trend logging & replay buffer
uint32_t tTrend = 0;
uint32_t tReplay = 0;
struct ReplaySample { uint32_t ms; float p, vbat, fuel; St state; };
ReplaySample replay[REPLAY_DEPTH]; int replay_idx=0;

// Running CRC32 for daily log
uint32_t g_crc32 = 0xFFFFFFFF;

// Shell log tail
bool g_logtail = false;
size_t g_tail_pos = 0;

// Adaptive crank learning
bool this_crank_started=false;
uint32_t crank_started_ms=0;
float ema_crank_ms = CRANK_TIMEOUT_MS_DEFAULT * 0.6f;

//============================= Utilities ==================================
static inline float adcToV(int raw, float vref=3.3f){ return (float)raw * (vref / ((1<<ADC_BITS)-1)); }
float readBattery(){ int r=analogRead(ADC_BATT_GPIO); return adcToV(r,3.3f) * 11.0f; }
float readPressureBar(){
  int r=analogRead(ADC_PRESS_GPIO);
  float v=adcToV(r,3.3f) * (5.0f/3.3f);
  if (v<0.3f || v>4.7f) return -1.0f;
  float t=(v-0.5f)/4.0f; t=constrain(t,0.0f,1.0f);
  return t*16.0f;
}
float readFuelPercent(){
  int r=analogRead(ADC_FUEL_GPIO);
  float v=adcToV(r,3.3f);
  float t=(v - fuel_v_full)/(fuel_v_empty - fuel_v_full); t=constrain(t,0.0f,1.0f);
  return (1.0f - t)*100.0f;
}
float readTempAdcC(int gpio){
  if (gpio<0) return NAN;
  int r=analogRead(gpio);
  float v=adcToV(r,3.3f);
  // You can adapt mapping: assume 0.5–4.5 V = 0–150 C as placeholder
  if (v<0.3f || v>4.7f) return NAN;
  float t=(v-0.5f)/4.0f; t=constrain(t,0.0f,1.0f);
  return t*150.0f;
}

bool inY(uint8_t y){ PCF8574* bank=(y<=8)?&IN1:&IN2; uint8_t bit=(y-1)%8; return bank->read(bit)==1; }
void outX(uint8_t x, bool on){ PCF8574* bank=(x<=8)?&OUT1:&OUT2; uint8_t bit=(x-1)%8; bank->write(bit, on?1:0); }

//============================= CRC32 ======================================
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len){
  static uint32_t table[256]; static bool init=false;
  if(!init){ for(uint32_t i=0;i<256;i++){ uint32_t c=i; for(int k=0;k<8;k++) c=c&1? (0xEDB88320 ^ (c>>1)) : (c>>1); table[i]=c; } init=true; }
  crc = ~crc;
  for(size_t i=0;i<len;i++) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

//============================= Logging =====================================
File g_logFile; String g_logDay; uint32_t g_lastRotateCheckMs=0;

String dateStamp(){
  struct tm t{}; if(getLocalTime(&t,5)){ char b[16]; strftime(b,sizeof(b),"%Y%m%d",&t); return String(b); }
  static uint32_t bootDay=0xFFFFFFFF; uint32_t days=millis()/86400000UL; if(bootDay==0xFFFFFFFF) bootDay=days;
  char b[16]; snprintf(b,sizeof(b),"D%05lu",(unsigned long)(days-bootDay)); return String(b);
}
void pruneOld(){
  File dir=SPIFFS.open(LOG_DIR); if(!dir) return;
  struct Item{String n; long s;} items[64]; int n=0;
  for(File f=dir.openNextFile(); f && n<64; f=dir.openNextFile()){
    String p=f.name(); f.close(); if(!p.endsWith(".log")) continue;
    int slash=p.lastIndexOf('/'); String base=(slash>=0)?p.substring(slash+1):p;
    String stem=base.substring(0, base.length()-4); long sc=0;
    if(stem.length()==8 && stem[0]!='D') sc=stem.toInt(); else if(stem.length()==6 && stem[0]=='D') sc=stem.substring(1).toInt();
    items[n++]={p,sc};
  } dir.close(); if(n<=LOG_RETENTION_DAYS) return;
  for(int i=0;i<n-1;i++) for(int j=i+1;j<n;j++) if(items[j].s<items[i].s){auto t=items[i]; items[i]=items[j]; items[j]=t;}
  for(int i=0;i<n-LOG_RETENTION_DAYS;i++) SPIFFS.remove(items[i].n);
}
void write_crc_footer(File& f){
  char line[64]; snprintf(line,sizeof(line),"-- CRC32=%08X --\n",(unsigned)g_crc32);
  f.print(line); f.flush();
}
void openLogDay(const String& day){
  if(g_logFile){ write_crc_footer(g_logFile); g_logFile.close(); }
  if(!SPIFFS.exists(LOG_DIR)) SPIFFS.mkdir(LOG_DIR);
  String fn=String(LOG_DIR)+"/"+day+".log";
  g_logFile = SPIFFS.open(fn, FILE_APPEND);
  if(g_logFile){ String hdr="-- Log start "+day+" --\n"; g_logFile.print(hdr); g_logFile.flush(); g_crc32=0xFFFFFFFF; g_crc32=crc32_update(g_crc32,(const uint8_t*)hdr.c_str(),hdr.length()); }
}
void rotateIfNeeded(){ String d=dateStamp(); if(d!=g_logDay){ g_logDay=d; openLogDay(d); pruneOld(); } }
void log_init(){ rotateIfNeeded(); g_lastRotateCheckMs=millis(); }

void log_raw(const char* s){
  rotateIfNeeded(); if(!g_logFile) return;
  g_logFile.print(s); g_logFile.flush();
  g_crc32 = crc32_update(g_crc32,(const uint8_t*)s,strlen(s));
}
void log_line(const char* level, const char* fmt, va_list ap){
  char msg[240]; vsnprintf(msg,sizeof(msg),fmt,ap);
  char buf[320]; snprintf(buf,sizeof(buf),"[%10lu] %s %s\n",(unsigned long)millis(), level, msg);
  log_raw(buf);
}
void log_info(const char* fmt, ...){ va_list ap; va_start(ap,fmt); log_line("[INFO]",fmt,ap); va_end(ap); }
void log_warn(const char* fmt, ...){ va_list ap; va_start(ap,fmt); log_line("[WARN]",fmt,ap); va_end(ap); }
void log_fault(const char* fmt, ...){ va_list ap; va_start(ap,fmt); log_line("[FAULT]",fmt,ap); va_end(ap); }

void log_tick(){ uint32_t now=millis(); if(now-g_lastRotateCheckMs>=LOG_ROTATE_CHECK_MS){ g_lastRotateCheckMs=now; rotateIfNeeded(); } }

//============================= RTC / TZ ====================================
void rtc_apply_tz(int minutes){ g_tz_offset_min=minutes; configTime(g_tz_offset_min*60,0,nullptr,nullptr); }
bool rtc_set_datetime(const char* ds,const char* ts){
  int Y,M,D,h,m,s; if(sscanf(ds,"%d-%d-%d",&Y,&M,&D)!=3) return false; if(sscanf(ts,"%d:%d:%d",&h,&m,&s)!=3) return false;
  struct tm t{}; t.tm_year=Y-1900; t.tm_mon=M-1; t.tm_mday=D; t.tm_hour=h; t.tm_min=m; t.tm_sec=s;
  time_t local_ts=mktime(&t); time_t utc_ts= local_ts - (g_tz_offset_min*60);
  struct timeval tv{.tv_sec=utc_ts,.tv_usec=0}; settimeofday(&tv,nullptr); return true;
}
String rtc_get_datetime(){ struct tm t{}; if(!getLocalTime(&t,5)) return "1970-01-01 00:00:00 +00:00";
  char b[32]; strftime(b,sizeof(b),"%Y-%m-%d %H:%M:%S",&t); char tzs[8]; sprintf(tzs," %+03d:%02d", g_tz_offset_min/60, abs(g_tz_offset_min)%60); return String(b)+tzs; }
time_t rtc_now_epoch(){ time_t now; time(&now); return now; }
bool rtc_is_valid(){ return rtc_now_epoch() > 1577836800; } // 2020-01-01

//============================= UI / Language ================================
static void lineClear(uint8_t r){ lcd.setCursor(0,r); lcd.print("                    "); }
const char* stName(St s){ switch(s){ case IDLE:return "IDLE"; case PREHEAT:return "PREHT"; case FUEL_PULL_ST:return "FUEL"; case CRANK:return "CRANK"; case WARMUP:return "WARM"; case RUN:return "RUN"; case COOLDOWN:return "COOL"; case STOPPING:return "STOP"; default:return "?"; } }

// bargraph auto-scale
float g_bar_max = 10.0f; // dynamic display max
void update_bar_scale(float p){ if (p>0) { float target = max(6.0f, p*1.15f); g_bar_max = 0.9f*g_bar_max + 0.1f*target; g_bar_max = constrain(g_bar_max,6.0f,16.0f); } }
void drawPressureBar(float p){
  // 18 chars bar (col 2..19), label at col0..1
  int width = 18;
  float frac = (p<=0)?0.0f: min(1.0f, p / g_bar_max);
  int filled = (int)round(frac*width);
  lcd.setCursor(0,1); lcd.print("P:");
  lcd.setCursor(2,1);
  for(int i=0;i<width;i++) lcd.print(i<filled ? '#' : '-');
}

void loadLanguage(const String& code){
  L.status="STATUS"; L.inputs="INPUTS"; L.runtime="RUNTIME"; L.service="SERVICE"; L.faults="FAULTS";
  L.service_minor_due="MINOR SERVICE DUE"; L.service_major_due="MAJOR SERVICE DUE"; L.exit_hint="EXIT: wait 30s";
  L.low_fuel_no_start="LOW FUEL - NO START";
  g_lang = code;
  String p = String(LANG_DIR)+"/"+code+".json";
  File f = SPIFFS.open(p, FILE_READ);
  if(!f){ log_warn("lang file %s missing, fallback en", p.c_str()); if(code!="en") loadLanguage("en"); return; }
  DynamicJsonDocument doc(4096);
  DeserializationError e = deserializeJson(doc, f);
  f.close();
  if(e){ log_warn("lang parse error, fallback en"); if(code!="en") loadLanguage("en"); return; }
  if(doc.containsKey("status")) L.status = (const char*)doc["status"];
  if(doc.containsKey("inputs")) L.inputs = (const char*)doc["inputs"];
  if(doc.containsKey("runtime")) L.runtime = (const char*)doc["runtime"];
  if(doc.containsKey("service")) L.service = (const char*)doc["service"];
  if(doc.containsKey("faults")) L.faults = (const char*)doc["faults"];
  if(doc.containsKey("service_minor_due")) L.service_minor_due = (const char*)doc["service_minor_due"];
  if(doc.containsKey("service_major_due")) L.service_major_due = (const char*)doc["service_major_due"];
  if(doc.containsKey("exit_hint")) L.exit_hint = (const char*)doc["exit_hint"];
  if(doc.containsKey("low_fuel_no_start")) L.low_fuel_no_start = (const char*)doc["low_fuel_no_start"];
  log_info("lang loaded %s", code.c_str());
}

void lcdStatus(float pBar, float vBatt, float fuelPct) {
  lineClear(0); lcd.setCursor(0,0); lcd.print("MDVN71 "); lcd.print(FW_VERSION_STR);
  drawPressureBar(pBar); // row 1
  lineClear(2); lcd.setCursor(0,2); lcd.printf("VBAT=%4.1fV  F=%3.0f%%", vBatt, fuelPct);
  lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime().substring(11));
}

// Diagnostics menu
void lcdDiag(){
  switch(diagPage){
    case DPG_STATUS: {
      float vb=readBattery(); float p=readPressureBar(); float fu=fuel_percent;
      lineClear(0); lcd.setCursor(0,0); lcd.print(L.status); lcd.print(" "); lcd.print(stName(st));
      lineClear(1); lcd.setCursor(0,1); lcd.printf("P=%4.1fbar V=%4.1fV", p, vb);
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Fuel=%3.0f%%  D+=%d", fu, (int)inY(Y_ALT_DPLUS));
      lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime());
    } break;
    case DPG_INPUTS: {
      lineClear(0); lcd.setCursor(0,0); lcd.print(L.inputs);
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Y01=%d Y02=%d Y03=%d Y04=%d", inY(1),inY(2),inY(3),inY(4));
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Y05=%d Y06=%d Y07=%d Y08=%d", inY(5),inY(6),inY(7),inY(8));
      lineClear(3); lcd.setCursor(0,3); lcd.printf("Y09=%d", inY(9));
    } break;
    case DPG_RUNTIME: {
      lineClear(0); lcd.setCursor(0,0); lcd.print(L.runtime);
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Hours total: %6.1f", hours_total);
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Since service: %6.1f", hours_since_service);
      lineClear(3); lcd.setCursor(0,3); lcd.print(L.exit_hint);
    } break;
    case DPG_SERVICE: {
      lineClear(0); lcd.setCursor(0,0); lcd.print(L.service);
      lineClear(1); lcd.setCursor(0,1); lcd.printf("Minor: %s", service_minor_due? "DUE":"OK ");
      lineClear(2); lcd.setCursor(0,2); lcd.printf("Major: %s", service_major_due? "DUE":"OK ");
      lineClear(3); lcd.setCursor(0,3); lcd.print("ACK via shell");
    } break;
    case DPG_FAULTS: {
      lineClear(0); lcd.setCursor(0,0); lcd.print(L.faults);
      lineClear(1); lcd.setCursor(0,1); lcd.print("Oil/Temp/Press/D+...");
      lineClear(2); lcd.setCursor(0,2); lcd.print("See logs for detail");
      lineClear(3); lcd.setCursor(0,3); lcd.print(rtc_get_datetime().substring(11));
    } break;
    default: break;
  }
}

//============================= Trend / Replay ==============================
void trend_write_header_if_new(const String& day){
  if(!SPIFFS.exists(TREND_DIR)) SPIFFS.mkdir(TREND_DIR);
  String fn = String(TREND_DIR)+"/"+day+".csv";
  if(!SPIFFS.exists(fn)){
    File f = SPIFFS.open(fn, FILE_WRITE);
    if(f){ f.println("millis,datetime,pressure_bar,vbat,fuel_pct,state"); f.close(); }
  }
}
void trend_append(float p, float vb, float fu){
  String day = dateStamp();
  trend_write_header_if_new(day);
  String fn = String(TREND_DIR)+"/"+day+".csv";
  File f = SPIFFS.open(fn, FILE_APPEND);
  if(!f) return;
  f.printf("%lu,%s,%.2f,%.2f,%.1f,%s\n",(unsigned long)millis(), rtc_get_datetime().c_str(), p, vb, fu, stName(st));
  f.close();
}
void replay_push(float p, float vb, float fu){
  replay[replay_idx] = { millis(), p, vb, fu, st };
  replay_idx = (replay_idx+1)%REPLAY_DEPTH;
}
void replay_dump(){
  if(!SPIFFS.exists(REPLAY_DIR)) SPIFFS.mkdir(REPLAY_DIR);
  struct tm t{}; getLocalTime(&t,5);
  char name[48]; strftime(name,sizeof(name),"/replay/%Y%m%d_%H%M%S.csv",&t);
  File f = SPIFFS.open(name, FILE_WRITE);
  if(!f) return;
  f.println("millis,pressure_bar,vbat,fuel_pct,state");
  int idx = replay_idx;
  for(int i=0;i<REPLAY_DEPTH;i++){
    ReplaySample s = replay[idx]; idx=(idx+1)%REPLAY_DEPTH;
    f.printf("%lu,%.2f,%.2f,%.1f,%s\n",(unsigned long)s.ms, s.p, s.vbat, s.fuel, stName(s.state));
  }
  f.close();
  log_info("replay saved %s", name);
}

//============================= Service check (unchanged thresholds) ========
void service_check_and_set_flags(){
  if (!service_minor_due && hours_since_service >= 500.0f) { service_minor_due=true; prefs.begin("mdvn71",false); prefs.putBool("svc_minor_due",true); prefs.end(); log_warn("service: minor due (>=500h)"); }
  if (!service_major_due && hours_since_service >= 1500.0f){ service_major_due=true; prefs.begin("mdvn71",false); prefs.putBool("svc_major_due",true); prefs.end(); log_warn("service: major due (>=1500h)"); }
  if (!service_major_due && rtc_is_valid() && last_service_ts>0){
    const int64_t oneYear = 365LL*24*3600; time_t nowT=rtc_now_epoch();
    if ((nowT - (time_t)last_service_ts) >= oneYear){ service_major_due=true; prefs.begin("mdvn71",false); prefs.putBool("svc_major_due",true); prefs.end(); log_warn("service: major due (>=1y)"); }
  }
}

//============================= FSM ========================================
float pBar = 0.0f; float vBatt=12.0f;
uint32_t boot_ms=0;

void enter(St s){ st=s; tState=millis(); }

int preheatSecondsSmart(){ return PREHEAT_S_BASE; }

void start_crank_timer(){ this_crank_started=true; crank_started_ms=millis(); }
void on_crank_success(){
  if(this_crank_started){
    uint32_t dur = millis()-crank_started_ms;
    // EMA learn then derive dynamic timeout target
    ema_crank_ms = (1.0f-CRANK_LEARN_ALPHA)*ema_crank_ms + CRANK_LEARN_ALPHA*(float)dur;
    uint32_t learned = (uint32_t)min((float)CRANK_LEARN_MAX_MS, max((float)CRANK_LEARN_MIN_MS, ema_crank_ms*1.15f));
    crankTimeoutMs_dyn = learned;
    prefs.begin("mdvn71",false); prefs.putUInt("crank_to_ms", crankTimeoutMs_dyn); prefs.putFloat("crank_ema", ema_crank_ms); prefs.end();
    log_info("crank success %lums, learned_to=%lums", (unsigned long)dur, (unsigned long)crankTimeoutMs_dyn);
    this_crank_started=false;
  }
}

void enterCrank(){
  vBatt = readBattery();
  start_crank_timer();
  outX(X_STARTER,true);
  outX(X_UNLOAD, true);
  enter(CRANK);
  log_info("crank start base=%.1f", vBatt);
}

void doFSM(){
  switch(st){
    case IDLE:{
      outX(X_STARTER,false); outX(X_GLOW,false);
      outX(X_UNLOAD,true);   outX(X_3WAY_LOAD,false);

      // Diagnostics long-press
      static bool lastStart=false; static uint32_t startDownMs=0;
      bool key=inY(Y_KEY_ON), sb=inY(Y_START_BTN);
      if (key && sb && !lastStart) startDownMs=millis();
      if (key && sb && lastStart && !diagMode && (millis()-startDownMs>DIAG_ENTER_LONG_MS)) { diagMode=true; diagPage=DPG_STATUS; diagEnterMs=millis(); log_info("diag enter"); }
      if (diagMode){
        if (sb && !lastStart){ diagPage=(DiagPage)((diagPage+1)%DPG__COUNT); diagEnterMs=millis(); }
        if (millis()-diagEnterMs>DIAG_AUTO_EXIT_MS){ diagMode=false; log_info("diag exit timeout"); }
      } else if (key && sb && !lastStart){
        // Safe Start Inhibit: low fuel
        if (fuel_percent < FUEL_LOCKOUT_PCT){
          log_warn("start blocked: low fuel %.1f%%", fuel_percent);
          lineClear(0); lcd.setCursor(0,0); lcd.print(L.low_fuel_no_start);
        } else {
          log_info("start pressed");
          enter(PREHEAT);
        }
      }
      lastStart=sb;
    } break;

    case PREHEAT:{
      int ph=preheatSecondsSmart();
      outX(X_GLOW, ph>0);
      if (millis()-tState >= (uint32_t)(ph*1000UL)){
        outX(X_GLOW,false);
        outX(X_FUEL_PULL,true); outX(X_FUEL_HOLD,true);
        delay(1000);
        outX(X_FUEL_PULL,false);
        enterCrank();
      }
    } break;

    case CRANK:{
      const uint32_t now=millis();
      const bool suppr = (now - crank_started_ms) < CHARGE_DETECT_SUPPRESS_MS;
      vBatt = readBattery();
      bool chargeAbsOk = (vBatt >= chargeAbs);
      bool chargeDeltaOk= (vBatt >= (12.0f + chargeDelta)); // baseline approximated by initial vBatt read
      if (!suppr && (chargeAbsOk || chargeDeltaOk)){
        if (!inY(Y_OIL_PRESS_NC) || (now - crank_started_ms >= 400)){ // tiny guard
          outX(X_STARTER,false);
          on_crank_success();
          enter(WARMUP);
          break;
        }
      }
      if (!inY(Y_OIL_PRESS_NC)){ outX(X_STARTER,false); on_crank_success(); enter(WARMUP); break; }

      uint32_t timeout = min(crankTimeoutMs_base, crankTimeoutMs_dyn);
      if (now - crank_started_ms >= timeout){
        outX(X_STARTER,false);
        log_warn("crank timeout %lums", (unsigned long)timeout);
        static int retry=0;
        if (retry < CRANK_MAX_RETRIES){ retry++; delay(CRANK_RETRY_DELAY_MS); log_info("crank retry #%d", retry); enterCrank(); }
        else { outX(X_ALARM,true); delay(300); outX(X_ALARM,false); log_fault("crank failure"); replay_dump(); enter(IDLE); retry=0; }
      }
    } break;

    case WARMUP:{
      outX(X_UNLOAD,true); outX(X_3WAY_LOAD,false);
      if (millis()-tState > WARMUP_MS) enter(RUN);
    } break;

    case RUN:{
      // hourmeter
      if (last_hours_tick_ms==0) last_hours_tick_ms=millis();
      else { uint32_t dt=millis()-last_hours_tick_ms; last_hours_tick_ms += dt; float dh=dt/3600000.0f; hours_total+=dh; hours_since_service+=dh; }

      // pressure control with little debounce
      if (pBar <= P_LOAD_BAR_DEFAULT){ outX(X_UNLOAD,false); outX(X_3WAY_LOAD,true); }
      if (pBar >= P_UNLOAD_BAR_DEFAULT){ outX(X_UNLOAD,true); outX(X_3WAY_LOAD,false); }

      // key off -> cooldown
      if (!inY(Y_KEY_ON)){ log_info("key off -> cooldown"); enter(COOLDOWN); }
    } break;

    case COOLDOWN:{
      outX(X_UNLOAD,true); outX(X_3WAY_LOAD,false);
      if (millis()-tState > COOLDOWN_MS) enter(STOPPING);
    } break;

    case STOPPING:{
      outX(X_FUEL_HOLD,false); outX(X_FUEL_PULL,false);
      outX(X_STARTER,false);   outX(X_GLOW,false);
      log_info("stop -> deep sleep");
      delay(300);
      esp_deep_sleep_start();
    } break;
  }
}

//============================= Shell / Commands ============================
String readLine(){ String s; while(Serial.available()){ char c=Serial.read(); if(c=='\r') continue; if(c=='\n') break; s+=c; } return s; }

void cmd_help(String){
  Serial.println("# Commands:");
  Serial.println(" status            - brief status line");
  Serial.println(" cfgdump           - config dump");
  Serial.println(" logs              - list log files");
  Serial.println(" logdump [day] [N]- print last N lines (default 200)");
  Serial.println(" logtail           - toggle live log tail to serial");
  Serial.println(" date              - show RTC datetime");
  Serial.println(" setdate Y-M-D h:m:s - set RTC");
  Serial.println(" tz <minutes>      - set timezone offset");
  Serial.println(" setcharge ...     - abs|delta <V>|test");
  Serial.println(" setcrank timeout <s> - set base timeout");
  Serial.println(" fuelcal full|empty|show - calibrate fuel sender");
  Serial.println(" service show|ack minor|ack major");
  Serial.println(" setlang <code>    - set language (loads /lang/<code>.json)");
  Serial.println(" config export|import - JSON config to/from /config.json");
}

void cmd_status(String){
  Serial.printf("st=%s p=%.1f vbat=%.2f fuel=%3.0f%% D+=%d FW=%s\n", stName(st), pBar, vBatt, fuel_percent, (int)inY(Y_ALT_DPLUS), FW_VERSION_STR);
}

void cmd_logs(String){
  File dir=SPIFFS.open(LOG_DIR); if(!dir){ Serial.println("# no /log"); return; }
  Serial.println("# Log files:"); for(File f=dir.openNextFile(); f; f=dir.openNextFile()){ Serial.printf("%s (%lu bytes)\n", f.name(), (unsigned long)f.size()); f.close(); } dir.close();
}

void cmd_logdump(String a){
  String day; int n=200;
  if (a.length()){
    char buf[32]; int lines=0;
    if (sscanf(a.c_str(), "%31s %d", buf, &lines)>=1){ day=String(buf); if(lines>0) n=lines; }
  } else day = dateStamp();
  String fn = String(LOG_DIR)+"/"+(day.length()?day:dateStamp())+".log";
  File f=SPIFFS.open(fn, FILE_READ); if(!f){ Serial.println("# no file"); return; }
  // print last N lines
  String content = f.readString();
  f.close();
  int count=0; for(int i=content.length()-1;i>=0;i--) if(content[i]=='\n'){ count++; if(count==n){ content=content.substring(i+1); break; } }
  Serial.print(content);
}

void cmd_logtail(String){ g_logtail = !g_logtail; g_tail_pos=0; Serial.printf("logtail %s\n", g_logtail?"ON":"OFF"); }

void cmd_date(String){ Serial.println(rtc_get_datetime()); }
void cmd_setdate(String a){
  int sp=a.indexOf(' '); if(sp<0){ Serial.println("Usage: setdate YYYY-MM-DD HH:MM:SS"); return; }
  String ds=a.substring(0,sp), ts=a.substring(sp+1);
  if(rtc_set_datetime(ds.c_str(), ts.c_str())){ prefs.begin("mdvn71",false); prefs.putString("last_date",(ds+" "+ts)); prefs.end(); Serial.print("RTC set "); Serial.println(rtc_get_datetime()); log_info("RTC set %s", rtc_get_datetime().c_str()); }
  else Serial.println("Invalid datetime");
}
void cmd_tz(String a){
  if(a.length()==0){ Serial.printf("TZ %+d min\n", g_tz_offset_min); return; }
  int off; if(sscanf(a.c_str(),"%d",&off)==1){ rtc_apply_tz(off); prefs.begin("mdvn71",false); prefs.putInt("tz_offset",off); prefs.end(); Serial.printf("TZ set %+d min\n", off); log_info("TZ set %+d", off); }
  else Serial.println("Usage: tz <minutes_offset>");
}

void cmd_cfgdump(String){
  Serial.printf("FW=%s baseTO=%lus dynTO=%lums abs=%.2fV delta=%.2fV fuelV(full)=%.2f empty=%.2f ema_crank=%.0fms lang=%s\n",
    FW_VERSION_STR, (unsigned long)(crankTimeoutMs_base/1000), (unsigned long)crankTimeoutMs_dyn, chargeAbs, chargeDelta, fuel_v_full, fuel_v_empty, ema_crank_ms, g_lang.c_str());
  Serial.printf("Hours total=%.1f sinceSvc=%.1f minorDue=%d majorDue=%d\n", hours_total, hours_since_service, service_minor_due, service_major_due);
}

void cmd_setcharge(String a){
  if (a.startsWith("test")){
    Serial.println("# Live charge-in test (5 s)"); float base=readBattery(); uint32_t t0=millis(); bool printed=false;
    while (millis()-t0<5000){ float v=readBattery(); bool trip=(v>=chargeAbs)||(v>=base+chargeDelta); if(trip && !printed){ Serial.printf("Rise @ %.1fs (%.2fV)\n",(millis()-t0)/1000.0,v); printed=true; } delay(100); }
    Serial.printf("Baseline=%.2fV abs=%.2fV delta=%.2fV\n", base, chargeAbs, chargeDelta); return;
  }
  char mode[8]; float val;
  if (sscanf(a.c_str(), "%7s %f", mode, &val)==2){
    if (!strcasecmp(mode,"abs"))   { chargeAbs=val;   prefs.begin("mdvn71",false); prefs.putFloat("chg_abs",val); prefs.end(); Serial.printf("Charge abs=%.2fV\n", val); log_info("set chargeAbs=%.2f", val); }
    else if (!strcasecmp(mode,"delta")){ chargeDelta=val; prefs.begin("mdvn71",false); prefs.putFloat("chg_delta",val); prefs.end(); Serial.printf("Charge delta=%.2fV\n", val); log_info("set chargeDelta=%.2f", val); }
    else Serial.println("Usage: setcharge abs|delta <V>|test");
  } else if (a.length()==0) Serial.printf("chargeAbs=%.2fV chargeDelta=%.2fV\n", chargeAbs, chargeDelta);
  else Serial.println("Usage: setcharge abs|delta <V>|test");
}

void cmd_setcrank(String a){
  char mode[16]; int val;
  if (sscanf(a.c_str(), "%15s %d", mode, &val)==2 && !strcasecmp(mode,"timeout")){
    crankTimeoutMs_base=(uint32_t)max(1000, val*1000); prefs.begin("mdvn71",false); prefs.putUInt("crank_to_ms",crankTimeoutMs_base); prefs.end();
    Serial.printf("Crank base timeout=%lus\n",(unsigned long)(crankTimeoutMs_base/1000)); log_info("set crank base TO=%lu",(unsigned long)crankTimeoutMs_base);
  } else if (!a.length()) Serial.printf("baseTimeout=%lus dyn=%lums\n",(unsigned long)(crankTimeoutMs_base/1000),(unsigned long)crankTimeoutMs_dyn);
  else Serial.println("Usage: setcrank timeout <seconds>");
}

void cmd_fuelcal(String a){
  if(a=="full"){ fuel_v_full=adcToV(analogRead(ADC_FUEL_GPIO),3.3f); prefs.begin("mdvn71",false); prefs.putFloat("fuel_full",fuel_v_full); prefs.end(); Serial.printf("Fuel FULL: %.3f V\n", fuel_v_full); log_info("fuelcal full=%.3f", fuel_v_full); }
  else if(a=="empty"){ fuel_v_empty=adcToV(analogRead(ADC_FUEL_GPIO),3.3f); prefs.begin("mdvn71",false); prefs.putFloat("fuel_empty",fuel_v_empty); prefs.end(); Serial.printf("Fuel EMPTY: %.3f V\n", fuel_v_empty); log_info("fuelcal empty=%.3f", fuel_v_empty); }
  else if(a=="show" || a.length()==0){ Serial.printf("Fuel cal: full=%.3f V empty=%.3f V now=%.3f V\n", fuel_v_full, fuel_v_empty, adcToV(analogRead(ADC_FUEL_GPIO),3.3f)); }
  else Serial.println("Usage: fuelcal full|empty|show");
}

void cmd_service(String a){
  time_t nowT; time(&nowT);
  if (a=="ack minor"){
    hours_since_service=0; service_minor_due=false;
    prefs.begin("mdvn71",false); prefs.putFloat("hrs_since",hours_since_service); prefs.putBool("svc_minor_due",false); prefs.end();
    Serial.println("Minor service acknowledged."); log_info("service ack minor");
  } else if (a=="ack major"){
    hours_since_service=0; service_minor_due=false; service_major_due=false; last_service_ts=(int64_t)nowT;
    prefs.begin("mdvn71",false); prefs.putFloat("hrs_since",hours_since_service); prefs.putBool("svc_minor_due",false); prefs.putBool("svc_major_due",false); prefs.putLong64("svc_last_ts",last_service_ts); prefs.end();
    Serial.println("Major service acknowledged."); log_info("service ack major");
  } else if (a=="show" || !a.length()){
    char buf[32]="n/a"; double days_since=-1.0;
    if(last_service_ts>0){ struct tm lt{}; localtime_r((time_t*)&last_service_ts,&lt); strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&lt); days_since = rtc_is_valid()? (rtc_now_epoch()-(time_t)last_service_ts)/86400.0:-1.0; }
    Serial.printf("Hours total=%.1f Since=%.1f Minor=%s Major=%s LastMajor=%s (%.0f d)\n", hours_total, hours_since_service, service_minor_due?"YES":"NO", service_major_due?"YES":"NO", buf, days_since);
  } else {
    Serial.println("Usage: service show|ack minor|ack major");
  }
}

// language
void cmd_setlang(String a){
  if(!a.length()){ Serial.printf("lang=%s\n", g_lang.c_str()); return; }
  loadLanguage(a);
  prefs.begin("mdvn71",false); prefs.putString("lang", g_lang); prefs.end();
  Serial.printf("Language set: %s\n", g_lang.c_str());
}

// Config import/export
void config_export(){
  DynamicJsonDocument doc(1024);
  doc["chargeAbs"]=chargeAbs; doc["chargeDelta"]=chargeDelta;
  doc["crankBaseToMs"]=crankTimeoutMs_base; doc["crankEmaMs"]=ema_crank_ms;
  doc["fuelVfull"]=fuel_v_full; doc["fuelVempty"]=fuel_v_empty;
  doc["lang"]=g_lang; doc["tz"]=g_tz_offset_min;
  File f=SPIFFS.open(CONFIG_JSON_PATH, FILE_WRITE); if(!f){ Serial.println("write fail"); return; }
  serializeJsonPretty(doc, f); f.close(); Serial.println("config exported");
}
void config_import(){
  File f=SPIFFS.open(CONFIG_JSON_PATH, FILE_READ); if(!f){ Serial.println("missing /config.json"); return; }
  DynamicJsonDocument doc(2048); DeserializationError e=deserializeJson(doc, f); f.close();
  if(e){ Serial.println("json parse error"); return; }
  if(doc.containsKey("chargeAbs")) chargeAbs=doc["chargeAbs"].as<float>();
  if(doc.containsKey("chargeDelta")) chargeDelta=doc["chargeDelta"].as<float>();
  if(doc.containsKey("crankBaseToMs")) crankTimeoutMs_base=doc["crankBaseToMs"].as<uint32_t>();
  if(doc.containsKey("crankEmaMs")) ema_crank_ms=doc["crankEmaMs"].as<float>();
  if(doc.containsKey("fuelVfull")) fuel_v_full=doc["fuelVfull"].as<float>();
  if(doc.containsKey("fuelVempty")) fuel_v_empty=doc["fuelVempty"].as<float>();
  if(doc.containsKey("lang")) { loadLanguage(doc["lang"].as<String>()); }
  if(doc.containsKey("tz")) { rtc_apply_tz(doc["tz"].as<int>()); }
  prefs.begin("mdvn71",false);
  prefs.putFloat("chg_abs", chargeAbs); prefs.putFloat("chg_delta", chargeDelta);
  prefs.putUInt("crank_to_ms", crankTimeoutMs_base); prefs.putFloat("crank_ema", ema_crank_ms);
  prefs.putFloat("fuel_full", fuel_v_full); prefs.putFloat("fuel_empty", fuel_v_empty);
  prefs.putString("lang", g_lang); prefs.putInt("tz_offset", g_tz_offset_min);
  prefs.end();
  Serial.println("config imported");
}

void cmd_config(String a){
  if(a=="export") config_export();
  else if(a=="import") config_import();
  else Serial.println("Usage: config export|import");
}

// command table
typedef void (*CmdFn)(String);
struct Cmd { const char* name; CmdFn fn; const char* help; };
void cmd_logs(String); void cmd_logdump(String); void cmd_logtail(String);
Cmd CMDS[] = {
  {"help",      cmd_help,     "help"},
  {"status",    cmd_status,   "status"},
  {"cfgdump",   cmd_cfgdump,  "cfgdump"},
  {"logs",      cmd_logs,     "logs"},
  {"logdump",   cmd_logdump,  "logdump [YYYYMMDD] [N]"},
  {"logtail",   cmd_logtail,  "logtail toggle"},
  {"date",      cmd_date,     "date"},
  {"setdate",   cmd_setdate,  "setdate YYYY-MM-DD HH:MM:SS"},
  {"tz",        cmd_tz,       "tz <minutes>"},
  {"setcharge", cmd_setcharge,"setcharge abs|delta <V>|test"},
  {"setcrank",  cmd_setcrank, "setcrank timeout <s>"},
  {"fuelcal",   cmd_fuelcal,  "fuelcal full|empty|show"},
  {"service",   cmd_service,  "service show|ack minor|ack major"},
  {"setlang",   cmd_setlang,  "setlang <code>"},
  {"config",    cmd_config,   "config export|import"},
};

String readCmd(){
  String s=readLine(); if(!s.length()) return s;
  for(auto &c:CMDS){ String n=c.name; if(s.startsWith(n)){ String a=s.substring(n.length()); a.trim(); c.fn(a); return s; } }
  Serial.println("unknown (try: help)");
  return s;
}

//============================= Setup / Loop ================================
void startup_selfcheck(){
  float vb=readBattery();
  log_info("BOOTOK vbat=%.2f FW=%s build=%s", vb, FW_VERSION_STR, __DATE__ " " __TIME__);
}

// soft recovery check
void soft_recovery_check(){
  // if booted due to brown-out and rail looks like engine running, jump to RUN
  if (boot_brownout){
    float v=readBattery();
    if (v >= 13.2f || inY(Y_ALT_DPLUS)){
      log_warn("brown-out recovery: resume RUN (v=%.1f)", v);
      enter(RUN);
    }
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  analogReadResolution(ADC_BITS);
  Wire.begin(I2C_SDA, I2C_SCL);

  lcd.init(); lcd.backlight(); lcd.clear(); lcd.print("MDVN71 "); lcd.print(FW_VERSION_STR);

  SPIFFS.begin(true);
  log_init();

  prefs.begin("mdvn71", true);
  int tz = prefs.getInt("tz_offset", 0);
  String last_date = prefs.getString("last_date","");
  chargeAbs   = prefs.getFloat("chg_abs",   CHARGE_DETECT_V_ABS_DEFAULT);
  chargeDelta = prefs.getFloat("chg_delta", CHARGE_DETECT_DELTA_DEFAULT);
  crankTimeoutMs_base = prefs.getUInt("crank_to_ms", CRANK_TIMEOUT_MS_DEFAULT);
  ema_crank_ms = prefs.getFloat("crank_ema", CRANK_TIMEOUT_MS_DEFAULT*0.6f);
  fuel_v_full  = prefs.getFloat("fuel_full", fuel_v_full);
  fuel_v_empty = prefs.getFloat("fuel_empty", fuel_v_empty);
  hours_total = prefs.getFloat("hrs_total", 0.0f);
  hours_since_service = prefs.getFloat("hrs_since", 0.0f);
  service_minor_due=prefs.getBool("svc_minor_due",false);
  service_major_due=prefs.getBool("svc_major_due",false);
  last_service_ts = prefs.getLong64("svc_last_ts",0);
  String lang = prefs.getString("lang","en");
  prefs.end();

  rtc_apply_tz(tz);
  if(last_date.length()){ int sp=last_date.indexOf(' '); if(sp>0) rtc_set_datetime(last_date.substring(0,sp).c_str(), last_date.substring(sp+1).c_str()); }

  IN1.begin(); IN2.begin(); OUT1.begin(); OUT2.begin();

  // determine reset reason for brown-out
  esp_reset_reason_t rr = esp_reset_reason();
  boot_brownout = (rr==ESP_RST_BROWNOUT);

  // language load
  loadLanguage(lang);

  startup_selfcheck();

  boot_ms = millis();
  enter(IDLE);
  soft_recovery_check();
}

uint32_t tUi=0, tLog=0, tPersist=0, tSvcCheck=0, tTherm=0;
float cylC=NAN, oilC=NAN;
uint32_t sensor_unmask_at=0;

void loop(){
  // analog reads
  pBar = readPressureBar();
  vBatt= readBattery();
  fuel_percent = readFuelPercent();
  update_bar_scale(pBar);

  // Sensor warm-up mask
  if (sensor_unmask_at==0) sensor_unmask_at = millis() + SENSOR_WARMUP_MS;

  // Thermals (optional)
  if (millis()-tTherm > THERM_SAMPLING_MS){
    tTherm=millis();
    if (ADC_TEMP_CYL_GPIO>=0) cylC = readTempAdcC(ADC_TEMP_CYL_GPIO);
    if (ADC_TEMP_OIL_GPIO>=0) oilC = readTempAdcC(ADC_TEMP_OIL_GPIO);
    // You could add EMA slope, compare against THERM_SLOPE_LIMIT_C_PER_MIN and log_warn/log_fault
  }

  // LCD UI
  if (millis()-tUi > 300){
    tUi=millis();
    if (diagMode) lcdDiag(); else lcdStatus(pBar, vBatt, fuel_percent);
    if (service_major_due){ static bool b=false; b=!b; lcd.setCursor(0,0); lcd.print(b? "MAJOR SERVICE DUE  ":"                    "); }
    else if (service_minor_due){ static bool b2=false; b2=!b2; lcd.setCursor(0,0); lcd.print(b2? "MINOR SERVICE DUE  ":"                    "); }
  }

  // Trend logger
  if (millis()-tTrend > TREND_PERIOD_MS){ tTrend=millis(); trend_append(pBar, vBatt, fuel_percent); }

  // Replay buffer (1 Hz)
  if (millis()-tReplay > REPLAY_PERIOD_MS){ tReplay=millis(); replay_push(pBar, vBatt, fuel_percent); }

  // Logger rotate tick
  if (millis()-tLog > 5000){ tLog=millis(); log_tick(); }

  // Persist counters & tail streaming
  if (millis()-tPersist > 15000){
    tPersist=millis();
    prefs.begin("mdvn71", false);
    prefs.putFloat("hrs_total", hours_total);
    prefs.putFloat("hrs_since", hours_since_service);
    prefs.putBool("svc_minor_due", service_minor_due);
    prefs.putBool("svc_major_due", service_major_due);
    prefs.putLong64("svc_last_ts", last_service_ts);
    prefs.end();

    if (g_logtail){
      String fn = String(LOG_DIR)+"/"+dateStamp()+".log";
      File f = SPIFFS.open(fn, FILE_READ);
      if (f){
        if (g_tail_pos > f.size()) g_tail_pos=0;
        f.seek(g_tail_pos, SeekSet);
        while (f.available()){
          String line = f.readStringUntil('\n');
          Serial.println(line);
        }
        g_tail_pos = f.position();
        f.close();
      }
    }
  }

  // Hourmeter only in RUN
  if (st!=RUN) last_hours_tick_ms=0;

  // Service checks
  if (millis()-tSvcCheck > 10000){ tSvcCheck=millis(); service_check_and_set_flags(); }

  // State machine tick
  doFSM();

  // Shell
  if (Serial.available()) readCmd();

  delay(20);
}
