# Rotair MDVN71 Diesel Compressor Controller – Firmware Feature List
**Firmware Version:** v4.19 PROD  
**Target Hardware:** ESP32 + KC868-A16  
**Date:** 2025-10-25  

This document lists all implemented firmware features grouped by subsystem, with purpose and implementation summaries for each.

## 1. Hardware Interaction & Safety Layer
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| KC868-A16 I2C Bus Integration | Interface ESP32 with 32 digital I/Os via dual PCF8574 expanders. | Uses two input banks (IN1/IN2) for Y01–Y09 and two output banks (OUT1/OUT2) for X01–X07; communication at 100 kHz on I2C bus (SDA = 4, SCL = 5). |
| MOSFET Output Drive with LED Parallel Indication | Provide direct 12 V signal control to relays/valves without mechanical relays. | X outputs drive MOSFETs; each channel includes built-in LED feedback for quick field diagnostics. |
| ADC Linearization for Pressure Transducer | Convert 0.5–4.5 V input into 0–16 bar with linear scaling. | Uses formula (v − 0.5)/4.0 × 16, with range clamping and fault detection if < 0.3 V or > 4.7 V. |
| Fuel Level Conversion (0–240 Ω Sender) | Translate variable-resistance fuel sender to calibrated fuel %. | 240 Ω reference resistor to 3.3 V; ADC node measured on CH2; linear interpolation between stored FULL and EMPTY voltages. |
| Input Debouncing for Active-High Digital Inputs | Prevent false triggers from switch bounce or noise. | Simple 10 ms median filtering per channel using short-term bit history. |
| Brown-Out Voltage Monitoring | Detect low-voltage conditions during crank or battery weakness. | Reads ADC34 scaled to 12 V rail; logs “brown-out” event when drop > 2 V within 200 ms. |
| Sensor Validation / Range Sanity | Identify disconnected or shorted sensors. | Continuous voltage sanity check (< 0.1 V or > 4.9 V for > 1 s → fault flag + log). |
| Over-Pressure Debounce Logic | Prevent pressure oscillation near unload threshold. | Requires sustained reading beyond limit for 500 ms before fault/unload transition. |
| I2C Bus Error Recovery | Protect against PCF8574 or LCD freeze after electrical noise. | Monitors transaction success; if > 1 s without ACK, re-init Wire bus and re-probe devices. |

## 2. Engine Control & Sequencing
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Key-On Wake / Key-Off Shutdown | System powers logic continuously but only runs when KEY-ON active. | Detects Y01 transition; on “off” triggers unload + 60 s cooldown then deep sleep. |
| Smart Preheat | Heat glow plugs prior to crank. | 5 s base delay (configurable); disables once engine is warm. |
| Fuel Pull & Hold Solenoid Sequencing | Properly actuate diesel fuel rack on start. | Pulses FUEL_PULL + FUEL_HOLD for 1 s, then holds H only while running. |
| Crank Control with Timeout and Retry | Prevent endless cranking if engine fails to fire. | Starts X03 (starter) up to 8 s; stops early on alternator voltage rise or oil-pressure OK; retries once after 3 s delay. |
| Alternator Voltage-Based Start Detection | Detect successful ignition even if D+ missing. | Monitors battery rise ≥ 13.2 V or +2 V from baseline for 300 ms → stop cranking. |
| D+ Redundancy Monitoring | Cross-check alternator output and D+ signal. | Logs warning if D+ ≠ expected given measured voltage. |
| Warm-Up Delay / Load Control | Allow engine stabilization before loading. | Keeps unload open for 30 s, then transitions to RUN with pressure-based load switching. |
| Load / Unload by Pressure | Maintain target pressure window. | Loads at 7 bar, unloads at 9 bar with 0.5 s debounce and hysteresis. |
| Cool-Down Sequence | Allow safe turbo cooldown before shutdown. | Runs unloaded for 60 s then stops fuel solenoid. |
| Deep Sleep Mode | Minimize idle current to protect 12 V battery. | Calls esp_deep_sleep_start() after COOLDOWN; wakes on KEY-ON high. |

## 3. Monitoring, Logging & Fault Handling
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Rolling Daily SPIFFS Logs | Maintain rotating event logs without SD card. | Writes /log/YYYYMMDD.log; auto-rotates every 24 h; keeps 7 days via filename date sort. |
| Structured Event Logging | Provide readable, timestamped trace. | log_write() prefix with [millis]; includes event type and data. |
| Startup Self-Check | Verify I2C devices and sensors on boot. | Confirms PCF8574 presence, ADC sanity, and battery voltage. |
| Fault Debounce and Auto-Re-Arm | Prevent transient fault latching. | Fault only if condition persists > 1 s; automatically clears after 10 s healthy. |
| RTC-Based Sleep Statistics | Preserve uptime and sleep duration metrics. | Uses RTC epoch stamps to calculate downtime for logs. |
| Runtime Heap/Stack Health Check | Early warning for memory exhaustion. | Periodically log free heap and stack watermark in background task. |
| Early Fault Suppression Window | Avoid false alarms during crank/transient. | Disables fault reporting during first 5 s after start sequence. |

## 4. Service Management & Hourmeter System
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Dual Hour Counters | Track cumulative and since-service runtime. | Integrates uptime only during RUN; persists in NVS every 15 s. |
| Minor/Major Service Flags | Distinguish small vs. full maintenance events. | Sets flags at 500 h (minor) and 1500 h (major) since last reset; stored in NVS. |
| 1-Year Calendar Timer | Ensure yearly major service even with low hours. | Compares RTC epoch to last major timestamp; triggers when ≥ 365 days. |
| Service Reminder on LCD | Alert operator when service due. | Flashes line 0 with “MINOR SERVICE DUE” or “MAJOR SERVICE DUE”. |
| Shell Commands for Maintenance | Provide CLI interface for technicians. | service show, service ack minor|major; updates NVS and logs. |
| Persistent Timestamp Storage | Record last major service time. | Saves Unix epoch in NVS for later delta computation. |
| Service Log File | Track maintenance history for audit. | Appends concise line in /service.log with date, hours, and ack type. |

## 5. User Interface & Diagnostics
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| 20x4 LCD Display (I2C) | Provide at-a-glance status to operator. | Displays state, pressure, voltage, fuel %, and clock. |
| Instant-On Backlight Policy | Ensure LCD visible immediately on wake. | Backlight enabled on boot and flashes on alarm. |
| Blink-on-Fault LED Output | External visual alarm indication. | Toggles X07 when fault active or alarm state. |
| Diagnostics Menu | Allow interactive inspection of system values. | Hold START > 5 s → enter menu; short-press cycles pages; exits after 30 s. |
| LCD Menu Pages | Quick field overview without PC. | Pages: Status, Input Mirror, Runtime, Service Flags, Faults. |
| Input Mirror Page | Visualize all Y inputs in real time. | Reads PCF8574 bits and displays 1/0 for Y01–Y09. |
| Runtime Page | Display operating hours summary. | Shows total and since-service hours. |
| Service Page | Display current service flag states. | Reads service_minor_due / service_major_due. |

## 6. Serial Shell & Configuration Interface
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Structured CLI Command Table | Simplify addition of commands. | CMDS[] array of {name, fn, help}; parsed linearly on input. |
| Configuration Commands | Adjust parameters without recompiling. | setcharge, setcrank, fuelcal, tz, setdate. |
| Diagnostic Commands | Retrieve runtime info and logs. | status, cfgdump, logs, date. |
| Live Alternator Test | Verify charge-in behavior in field. | setcharge test runs 5 s ADC sampling, prints detection events. |
| Fuel Calibration Utility | Calibrate sender voltage limits easily. | fuelcal full|empty|show stores ADC voltage references. |
| RTC & Timezone Configuration | Maintain accurate timestamps. | setdate YYYY-MM-DD HH:MM:SS and tz <offset_min>; persists to NVS. |
| Persistent Parameter Store (NVS) | Non-volatile configuration retention. | All tuning parameters stored under namespace mdvn71. |

## 7. Data Persistence & Filesystem
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| SPIFFS File System | On-board non-volatile storage for logs/config. | Auto-formats if missing; uses internal flash partition. |
| Daily Log Rotation & Retention | Keep storage tidy automatically. | Retains 7 days; deletes oldest via date sort. |
| Compacted Log Rotation | Prevents fragmentation or overflow. | Closes and reopens log when date changes; prunes old. |
| Config Shadow Copy | Maintain readable config backup. | JSON file in /config.json written on each settings save. |

## 8. System Reliability & Recovery
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Watchdog Timer Integration | Prevent lock-up from code hang. | Uses ESP32 hardware watchdog (enabled via SDKconfig). |
| Early Fault Suppression Window | Ignore transient conditions during crank. | 5 s mask after start before evaluating alarms. |
| Auto Fault Re-Arm | Clear faults once condition disappears. | Monitors all active faults; clears if > 10 s normal. |
| Crank Fault Recovery Path | Handle failed start attempts gracefully. | 1 retry after delay; logs fault after repeated failure. |
| Brown-Out Recovery Handling | Resume cleanly after voltage collapse. | On reboot, detect brown-out reset reason; resume to safe IDLE. |
| RTC-Based Sleep Stats | Compute true uptime vs. sleep ratio. | Compare boot time to last shutdown timestamp. |
| Self-Versioning Firmware | Display firmware version and build info automatically. | FW_VERSION_STR + __DATE__ __TIME__ printed on LCD and serial. |

## 9. Expandability Hooks
| Feature | Purpose | Implementation Summary |
|----------|----------|-------------------------|
| Scheduler Abstraction (Task Framework) | Provide modular time-based task control. | Cooperative loop; planned upgrade to tagged task scheduler. |
| Structured Fault Enumeration (FMS) | Centralize error codes for future telemetry. | Defines enum FaultCode; logs numeric + string cause. |
| Shell Command Expansion Interface | Simplify adding custom field commands. | Generic parser allows easy registration of new CMDS entries. |
| USB Diagnostic Mode (planned) | Enable serial console via USB CDC. | Hooks ready; activation in v4.20. |
| SPIFFS Log Compaction (future) | Optimize flash wear. | Merge small logs nightly; compress older days. |
| Smart Sleep Voting (planned) | Multi-task consent before deep sleep. | Boolean vote map ensures all modules ready before sleep. |
