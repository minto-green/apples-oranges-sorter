/*
 * ESP32-S3 + TMC2209 Stepper Control — 5-DOF Robotic Arm (CycloiDrive)
 * APPLES & ORANGES SORTER EDITION
 *
 * CHANGES vs ARM FIRMWARE 62226 (marked with [SORTER]):
 *   1. ONE ORDER AT A TIME — /api/sequence is REJECTED (HTTP 409 "BUSY")
 *      while a previous sequence is still queued or executing. The arm can
 *      physically never stack two sort orders. STOP and CLEAR are accepted
 *      ANY TIME via /api/override: STOP halts instantly and drains the
 *      queue; CLEAR drains the queue but lets the current move finish.
 *   2. ORDER-ID DEDUP — the brain sends "ID:S2#7" as the first line of every
 *      sort (the segment generation ID from the vision system). A repeat of
 *      the same ID is acknowledged with "DUP" and IGNORED, so a retry or
 *      double-send can never make the arm grab at an empty spot.
 *   3. /api/status now reports the current/last order ID and queue depth.
 *   4. Naming: this is a 5-DOF arm + gripper. Six driver channels remain
 *      wired/configured (J6 unused) so the pin map is unchanged.
 *
 * WIRING GUIDE (unchanged):
 * =============================================================================
 * DRIVER 1: STEP: GPIO05 | DIR: GPIO06 | MS1: GND, MS2: GND (Addr 0b00)
 * DRIVER 2: STEP: GPIO07 | DIR: GPIO08 | MS1: VCC, MS2: GND (Addr 0b01)
 * DRIVER 3: STEP: GPIO09 | DIR: GPIO10 | MS1: GND, MS2: VCC (Addr 0b10)
 * DRIVER 4: STEP: GPIO12 | DIR: GPIO13 | MS1: VCC, MS2: VCC (Addr 0b11)
 * DRIVER 5: STEP: GPIO14 | DIR: GPIO15 | MS1: GND, MS2: GND (Addr 0b00)
 * DRIVER 6: STEP: GPIO16 | DIR: GPIO17 | MS1: VCC, MS2: GND (Addr 0b01)  [unused J6]
 *
 * SHARED: EN_CTRL (Enable All): GPIO04
 * PERIPHERALS: RPI_RX: GPIO01 | RPI_TX: GPIO02 | SERVO: GPIO18 | DT: GPIO21
 * =============================================================================
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <queue>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "dashboard.h"
#include "ik_dashboard.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define DEBUG_MODE 0 // 1 = Serial logs (native USB can add delays)

// --- REMOTE LOG MIRROR ---
// Serial is USB-only, so it goes dark once the cable is pulled. Everything
// printed through Log is tee'd into this RAM ring buffer as well, and served
// over WiFi at /log. Oldest bytes are overwritten once the buffer is full.
#define LOG_BUF_SIZE 4096
static char logBuf[LOG_BUF_SIZE];
static uint32_t logTotal = 0;  // bytes ever written; doubles as the read cursor
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;  // both cores log

class LogMirror : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    portENTER_CRITICAL(&logMux);
    for (size_t i = 0; i < n; i++) logBuf[(logTotal + i) % LOG_BUF_SIZE] = (char)b[i];
    logTotal += n;
    portEXIT_CRITICAL(&logMux);
    return Serial.write(b, n);
  }
};
LogMirror Log;

String logSince(uint32_t &cursor) {
  portENTER_CRITICAL(&logMux);
  uint32_t total = logTotal;
  portEXIT_CRITICAL(&logMux);
  uint32_t oldest = (total > LOG_BUF_SIZE) ? total - LOG_BUF_SIZE : 0;
  if (cursor < oldest || cursor > total) cursor = oldest;
  String out;
  out.reserve(total - cursor);
  for (uint32_t i = cursor; i < total; i++) out += logBuf[i % LOG_BUF_SIZE];
  cursor = total;
  return out;
}

#if DEBUG_MODE
  #define DEBUG_PRINT(x) Log.print(x)
  #define DEBUG_PRINTLN(x) Log.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// --- WIFI DASHBOARD CREDENTIALS ---
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PW";
// mDNS name: reachable at http://ao-arm.local regardless of the DHCP lease,
// so the brain never needs a hardcoded IP.
const char* mdnsHost = "ao-arm";
WebServer server(80);

#define NUM_JOINTS 6   // 5-DOF arm + gripper; J6 channel wired but unused

// --- PIN DEFINITIONS ---
const uint8_t STEP_PINS[NUM_JOINTS] = {5, 7, 9, 12, 14, 16};
const uint8_t DIR_PINS[NUM_JOINTS]  = {6, 8, 10, 13, 15, 17};

#define EN_CTRL_PIN   4
#define RPI_RX_PIN    1
#define RPI_TX_PIN    2
#define SERVO_PIN     18
#define DT_PIN        21

// --- SERVO CALIBRATION ---
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

Servo toolheadServo;
unsigned long servoActionTimer = 0;
bool servoActive = false;

// --- TMC2209 SETTINGS ---
#define R_SENSE          0.11f
#define MOTOR_CURRENT_MA 800

// --- SEQUENCE COMMAND QUEUE ---
enum StepType { MOVE, DELAY, SET_SPEED, SET_ACCEL };
struct SequenceStep {
  StepType type;
  float angles[NUM_JOINTS];
  char toolheadCmd; // 'C' close, 'O' open, 'X' none
  unsigned long delayMs;
  float param;
};
std::queue<SequenceStep> commandQueue;
SemaphoreHandle_t queueMutex;

volatile bool isExecuting = false;
volatile bool homingRequested = false;
volatile bool stopRequested = false;
volatile bool clearRequested = false;   // [SORTER] drop queue, finish current move
volatile bool isHalted = false;
SequenceStep currentStep;
unsigned long delayStartTime = 0;

// [SORTER] --- ORDER TRACKING (one order at a time + dedup) ---
String currentOrderId = "";   // order now queued/executing
String lastOrderId    = "";   // last COMPLETED order (dedup reference)
volatile bool orderActive = false;

AccelStepper steppers[NUM_JOINTS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[3], DIR_PINS[3]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[4], DIR_PINS[4]),
  AccelStepper(AccelStepper::DRIVER, STEP_PINS[5], DIR_PINS[5])
};

// --- MECHANICAL LIMITS & MATH ---
const float GEAR_RATIO = 26.0;
// J1(0,0)=8, J2(1,0)=32, J3(0,1)=64, J4(1,1)=16, J5(0,0)=8, J6(1,0)=32
const uint16_t JOINT_MICROSTEPS[NUM_JOINTS] = {32, 64, 128, 16, 32, 64}; // CALIBRATED, dont change
float stepsPerDegree[NUM_JOINTS];

const float UNIVERSAL_MAX_SPEED_DEG_PER_SEC = 50.0;
const float UNIVERSAL_ACCEL_DEG_PER_SEC_SQ = 25.0;
// [SORTER FIX v2] The old flat MAX_STEP_RATE (12000 steps/s) applied the SAME
// raw step-rate ceiling to all six joints, even though they need wildly
// different step rates to hit the SAME angular speed (231 vs 1849
// steps/deg depending on microstepping). That meant J3 (128 microsteps)
// needed ~92,400 steps/s to reach the advertised 50 deg/s and got clamped
// down to ~6.5 deg/s, while J4 (16 microsteps) sailed through untouched at
// the full 50 deg/s. Whichever joint had the most microsteps in a given
// move became the bottleneck the whole sequence had to wait on — that's
// why the arm looked slow no matter what speed you sent.
//
// Fix: give every joint its OWN ceiling, sized to what IT needs to reach
// UNIVERSAL_MAX_SPEED_DEG_PER_SEC, instead of one shared number. Filled in
// setup() once stepsPerDegree[] is known. SAFETY_STEP_RATE_CEILING stays
// as a hard sanity bound against a corrupted command — it is NOT meant to
// be the everyday limiter anymore.
//
// !! BENCH-TEST BEFORE TRUSTING AT FULL SPEED !! This raises J3's ceiling
// ~7.7x (12,000 -> ~92,400 steps/s). Run a sort, listen for a grinding /
// stalling buzz on any joint, then home and confirm it lands on true zero
// with no drift. If one joint stalls, lower ONLY that joint's entry in
// MAX_STEP_RATE_PER_JOINT below — don't drag the whole array back down.
const float SAFETY_STEP_RATE_CEILING = 150000.0; // hard sanity bound, not a tuning knob
float MAX_STEP_RATE_PER_JOINT[NUM_JOINTS];        // set in setup()
const float MIN_STEP_ACCEL  = 100.0;    // steps/s^2 floor
const float SOFTWARE_LIMIT_DEG[NUM_JOINTS] = {360.0, 100.0, 100.0, 100.0, 360.0, 100.0};

float currentSequenceSpeed = UNIVERSAL_MAX_SPEED_DEG_PER_SEC;
float currentSequenceAccel = UNIVERSAL_ACCEL_DEG_PER_SEC_SQ;

// [SORTER] helper: is the arm free to accept a new order?
bool armIsFree() {
  bool queueEmpty;
  xSemaphoreTake(queueMutex, portMAX_DELAY);
  queueEmpty = commandQueue.empty();
  xSemaphoreGive(queueMutex);
  return queueEmpty && !isExecuting;
}

// --- NETWORK INFO ---
// Uses Log directly (not DEBUG_PRINT) so it works with DEBUG_MODE 0.
void printNetworkInfo() {
  if (WiFi.status() != WL_CONNECTED) {
    Log.println("\nWiFi: NOT CONNECTED");
    return;
  }
  Log.print("\nWiFi connected to: "); Log.println(WiFi.SSID());
  Log.print("DASHBOARD IP ADDRESS: http://"); Log.println(WiFi.localIP());
  Log.print("HOSTNAME: http://"); Log.print(mdnsHost); Log.println(".local");
  Log.print("Signal (RSSI): "); Log.print(WiFi.RSSI()); Log.println(" dBm");
}

// --- COMMAND PARSER (Shared by Serial & Web) ---
// enqueueCommandLocked: caller MUST already hold queueMutex.
// enqueueCommand: takes the mutex itself (Serial path, overrides).
void enqueueCommandLocked(const char* cmd);

void enqueueCommand(const char* cmd) {
  // STOP / IP are flag-only (no queue access) — handle before locking.
  if (strcasecmp(cmd, "STOP") == 0 || strcasecmp(cmd, "IP") == 0) {
    enqueueCommandLocked(cmd);
    return;
  }
  xSemaphoreTake(queueMutex, portMAX_DELAY);
  enqueueCommandLocked(cmd);
  xSemaphoreGive(queueMutex);
}

void enqueueCommandLocked(const char* cmd) {
  char inputBuffer[128];
  strncpy(inputBuffer, cmd, sizeof(inputBuffer) - 1);
  inputBuffer[sizeof(inputBuffer) - 1] = '\0';

  // STOP
  if (strcasecmp(inputBuffer, "STOP") == 0) {
    DEBUG_PRINTLN("STOP Requested...");
    stopRequested = true;
    isHalted = true;
    return;
  }

  // IP — report network info; not a motion command, must not clear the halt.
  if (strcasecmp(inputBuffer, "IP") == 0) {
    printNetworkInfo();
    return;
  }

  isHalted = false;

  // HO — Home & clear
  if (strcasecmp(inputBuffer, "HO") == 0) {
    DEBUG_PRINTLN("Homing Requested...");
    homingRequested = true;
  }
  // GH — Go Home
  else if (strcasecmp(inputBuffer, "GH") == 0) {
    DEBUG_PRINTLN("Queued GO HOME command.");
    SequenceStep s;
    s.type = MOVE;
    for (int i = 0; i < NUM_JOINTS; i++) s.angles[i] = 0.0;
    s.toolheadCmd = 'X';
    commandQueue.push(s);
  }
  // D:ms
  else if ((inputBuffer[0] == 'D' || inputBuffer[0] == 'd') && inputBuffer[1] == ':') {
    unsigned long d = atol(&inputBuffer[2]);
    SequenceStep s;
    s.type = DELAY;
    s.delayMs = d;
    commandQueue.push(s);
    DEBUG_PRINT("Queued DELAY: "); DEBUG_PRINT(d); DEBUG_PRINTLN(" ms");
  }
  // S:val
  else if ((inputBuffer[0] == 'S' || inputBuffer[0] == 's') && inputBuffer[1] == ':') {
    SequenceStep s;
    s.type = SET_SPEED;
    s.param = atof(&inputBuffer[2]);
    commandQueue.push(s);
    DEBUG_PRINT("Queued SPEED: "); DEBUG_PRINTLN(s.param);
  }
  // A:val
  else if ((inputBuffer[0] == 'A' || inputBuffer[0] == 'a') && inputBuffer[1] == ':') {
    SequenceStep s;
    s.type = SET_ACCEL;
    s.param = atof(&inputBuffer[2]);
    commandQueue.push(s);
    DEBUG_PRINT("Queued ACCEL: "); DEBUG_PRINTLN(s.param);
  }
  // 7-part CSV: J1..J6,T
  else {
    int commaCount = 0;
    for (int i = 0; inputBuffer[i] != '\0'; i++) {
      if (inputBuffer[i] == ',') commaCount++;
    }

    if (commaCount == 6) {
      SequenceStep s;
      s.type = MOVE;

      char* saveptr;
      char* token = strtok_r(inputBuffer, ",", &saveptr);
      int idx = 0;
      while (token != NULL && idx < NUM_JOINTS) {
        float parsedAngle = atof(token);
        s.angles[idx] = constrain(parsedAngle, -SOFTWARE_LIMIT_DEG[idx], SOFTWARE_LIMIT_DEG[idx]);
        idx++;
        token = strtok_r(NULL, ",", &saveptr);
      }

      if (token != NULL) s.toolheadCmd = token[0];
      else s.toolheadCmd = 'X';

      commandQueue.push(s);
      DEBUG_PRINTLN("Queued MOVE command.");
    } else {
      DEBUG_PRINTLN("Invalid format. Expected: J1..J6,T | D:ms | S:val | A:val | GH | HO");
    }
  }
}

// --- REMOTE LOG VIEWER PAGE ---
static const char log_page_html[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Arm Serial Log</title><style>
body{margin:0;background:#111;color:#ddd;font-family:sans-serif}
h1{font-size:14px;padding:10px;margin:0;background:#1b1b1b;border-bottom:1px solid #333}
#s{float:right;font-weight:400;color:#888}
pre{margin:0;padding:10px;height:calc(100vh - 40px);overflow:auto;
    font-family:Consolas,monospace;font-size:12px;white-space:pre-wrap;word-break:break-word}
</style></head><body>
<h1>Arm Serial Log <span id="s">connecting...</span></h1>
<pre id="o"></pre>
<script>
let cursor=0,o=document.getElementById('o'),s=document.getElementById('s');
async function poll(){
  try{
    const r=await fetch('/api/log?since='+cursor);
    cursor=r.headers.get('X-Log-Next')||cursor;
    const t=await r.text();
    if(t){
      const atEnd=o.scrollTop+o.clientHeight>=o.scrollHeight-30;
      o.textContent+=t;
      if(atEnd)o.scrollTop=o.scrollHeight;
    }
    s.textContent='live';
  }catch(e){s.textContent='disconnected';}
  setTimeout(poll,1000);
}
poll();
</script></body></html>
)HTML";

// --- WEB SERVER API ---
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", dashboard_html);
  });

  server.on("/ik", HTTP_GET, []() {
    server.send(200, "text/html", ik_dashboard_html);
  });

  // [SORTER] /api/sequence — ONE ORDER AT A TIME + ID DEDUP
  server.on("/api/sequence", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "No payload");
      return;
    }
    String payload = server.arg("plain");

    // Extract optional first line "ID:<order id>"
    String orderId = "";
    int firstNl = payload.indexOf('\n');
    String firstLine = (firstNl == -1) ? payload : payload.substring(0, firstNl);
    firstLine.trim();
    if (firstLine.startsWith("ID:") || firstLine.startsWith("id:")) {
      orderId = firstLine.substring(3);
      orderId.trim();
      payload = (firstNl == -1) ? "" : payload.substring(firstNl + 1);
    }

    // Dedup: an order we already ran (or are running) is acknowledged, not re-queued.
    if (orderId.length() > 0 &&
        (orderId == lastOrderId || (orderActive && orderId == currentOrderId))) {
      DEBUG_PRINT("[WEB] Duplicate order ignored: "); DEBUG_PRINTLN(orderId);
      server.send(200, "text/plain", "DUP");
      return;
    }

    // Busy-reject: never stack orders. (STOP/HO/GH bypass via /api/override.)
    if (!armIsFree()) {
      DEBUG_PRINTLN("[WEB] REJECTED — arm busy with an order.");
      server.send(409, "text/plain", "BUSY");
      return;
    }

    DEBUG_PRINTLN("\n[WEB] Accepted New Sequence Payload:");

    // [SORTER FIX] ATOMIC ENQUEUE. The old code enqueued line-by-line while
    // the motor core was already popping — for a moment mid-enqueue the queue
    // could read EMPTY, so /api/status reported IDLE while the pick was still
    // arriving and the order was falsely marked complete. The brain's
    // wait-for-idle then returned early and judged the sort while the arm was
    // still moving. Now: parse everything first with the queue LOCKED, set
    // the order id inside the same critical section, release once.
    xSemaphoreTake(queueMutex, portMAX_DELAY);
    if (orderId.length() > 0) {
      currentOrderId = orderId;
      orderActive = true;
    }
    int lineStart = 0;
    while (lineStart <= (int)payload.length()) {
      int lineEnd = payload.indexOf('\n', lineStart);
      String line = (lineEnd == -1) ? payload.substring(lineStart)
                                    : payload.substring(lineStart, lineEnd);
      line.trim();
      if (line.length() > 0) enqueueCommandLocked(line.c_str());
      if (lineEnd == -1) break;
      lineStart = lineEnd + 1;
    }
    xSemaphoreGive(queueMutex);

    server.send(200, "text/plain", "OK");
  });

  // [SORTER] /api/override — STOP / HO / GH always accepted (safety path)
  server.on("/api/override", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "No payload");
      return;
    }
    String cmd = server.arg("plain");
    cmd.trim();
    if (cmd.equalsIgnoreCase("CLEAR")) {
      // [SORTER] drop everything pending; the in-progress move finishes
      // safely (no mid-air freeze), then the arm is idle and order-free.
      clearRequested = true;
      orderActive = false;
      currentOrderId = "";
      server.send(200, "text/plain", "OK");
    } else if (cmd.equalsIgnoreCase("STOP") || cmd.equalsIgnoreCase("HO") ||
               cmd.equalsIgnoreCase("GH")) {
      enqueueCommand(cmd.c_str());
      orderActive = false;         // an override cancels the order context
      currentOrderId = "";
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Only STOP/CLEAR/HO/GH allowed here");
    }
  });

  server.on("/api/status", HTTP_GET, []() {
    int queueDepth;
    xSemaphoreTake(queueMutex, portMAX_DELAY);
    queueDepth = commandQueue.size();
    xSemaphoreGive(queueMutex);

    String json = "{\"status\": \"";
    if (isHalted) json += "HALTED";
    else if (isExecuting || queueDepth > 0) json += "MOVING";
    else json += "IDLE";
    json += "\", \"order\": \"";
    json += (orderActive ? currentOrderId : lastOrderId);
    json += "\", \"queue\": ";
    json += String(queueDepth);
    json += ", \"angles\": [";
    for (int i = 0; i < NUM_JOINTS; i++) {
      float angle = (float)steppers[i].currentPosition() / stepsPerDegree[i];
      json += String(angle, 1);
      if (i < NUM_JOINTS - 1) json += ", ";
    }
    json += "]}";
    server.send(200, "application/json", json);
  });

  // Remote serial monitor: reads the log ring buffer, no USB cable needed.
  server.on("/log", HTTP_GET, []() {
    server.send(200, "text/html", log_page_html);
  });

  server.on("/api/log", HTTP_GET, []() {
    uint32_t cursor = server.hasArg("since") ? (uint32_t)strtoul(server.arg("since").c_str(), NULL, 10) : 0;
    String text = logSince(cursor);
    server.sendHeader("X-Log-Next", String(cursor));
    server.send(200, "text/plain", text);
  });

  server.on("/api/ip", HTTP_GET, []() {
    String json = "{\"ip\": \"";
    json += WiFi.localIP().toString();
    json += "\", \"ssid\": \"";
    json += WiFi.SSID();
    json += "\", \"rssi\": ";
    json += String(WiFi.RSSI());
    json += ", \"mac\": \"";
    json += WiFi.macAddress();
    json += "\"}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

// --- CORE 0: COMMUNICATIONS TASK ---
void commsTask(void *pvParameters) {
  for (;;) {
    server.handleClient();

    while (Serial.available() > 0) {
      static char inputBuffer[128];
      static uint8_t bufIdx = 0;
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (bufIdx > 0) {
          inputBuffer[bufIdx] = '\0';
          enqueueCommand(inputBuffer);
          bufIdx = 0;
        }
      } else if (bufIdx < sizeof(inputBuffer) - 1) {
        inputBuffer[bufIdx++] = c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);
  // Without a USB host attached, CDC writes would otherwise block until they
  // time out and stall the caller. Drop them instead; /log still has the text.
  Serial.setTxTimeoutMs(0);

  unsigned long startWait = millis();
  while (!Serial && millis() - startWait < 2000) delay(10);

  Serial.println("\n--- ESP32-S3 5-DOF CycloiDrive Arm Controller (Sorter Edition) ---");

  pinMode(EN_CTRL_PIN, OUTPUT);
  digitalWrite(EN_CTRL_PIN, HIGH); // HIGH disables TMC2209 output

  Serial.println("Waiting 3 seconds for safety buffer before activating motors...");
  delay(3000);
  digitalWrite(EN_CTRL_PIN, LOW);
  Serial.println("Motors Activated.");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  toolheadServo.setPeriodHertz(50);
  toolheadServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servoActive = true;
  servoActionTimer = millis();

  queueMutex = xSemaphoreCreateMutex();

  Serial.println("TMC2209 UART is disabled. Using MS1/MS2 hardware microsteps.");

  for(int i = 0; i < NUM_JOINTS; i++) {
    stepsPerDegree[i] = (200.0 * JOINT_MICROSTEPS[i] * GEAR_RATIO) / 360.0;
    MAX_STEP_RATE_PER_JOINT[i] = min(UNIVERSAL_MAX_SPEED_DEG_PER_SEC * stepsPerDegree[i],
                                      SAFETY_STEP_RATE_CEILING);
    steppers[i].setMaxSpeed(MAX_STEP_RATE_PER_JOINT[i]);
    steppers[i].setAcceleration(UNIVERSAL_ACCEL_DEG_PER_SEC_SQ * stepsPerDegree[i]);
    steppers[i].setPinsInverted(false, false, false);
    steppers[i].setCurrentPosition(0);
  }

  Serial.println("\nSystem Ready.");
  Serial.println("Command Mode: ONE ORDER AT A TIME. Send 'ID:<order>' as first line.");
  Serial.println("Overrides accepted anytime: STOP (halt+drain) | CLEAR (drain) | HO | GH");
  Serial.println("Usage: J1..J6,T (T=C,O,X) | D:ms | GH (Go Home) | HO (Home & Clear)");

  Log.print("\nConnecting to ");
  Log.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Log.print(".");
  }
  if (MDNS.begin(mdnsHost)) {
    MDNS.addService("http", "tcp", 80);
  } else {
    Log.println("mDNS start FAILED — reach the arm by IP.");
  }

  printNetworkInfo();
  Log.println("Send 'IP' over Serial, or open http://<ip>/log to read this remotely.");

  setupWebServer();

  xTaskCreatePinnedToCore(commsTask, "CommsTask", 8192, NULL, 1, NULL, 0);
}

// --- CORE 1: MOTOR CONTROL LOOP ---
void loop() {
  if (clearRequested) {                 // [SORTER] queue drain, motion-safe
    xSemaphoreTake(queueMutex, portMAX_DELAY);
    while (!commandQueue.empty()) commandQueue.pop();
    xSemaphoreGive(queueMutex);
    clearRequested = false;
    DEBUG_PRINTLN("CLEAR executed: pending queue dropped.");
  }

  if (stopRequested) {
    xSemaphoreTake(queueMutex, portMAX_DELAY);
    while (!commandQueue.empty()) commandQueue.pop();
    xSemaphoreGive(queueMutex);

    for (int i = 0; i < NUM_JOINTS; i++) {
      steppers[i].moveTo(steppers[i].currentPosition());
    }
    isExecuting = false;
    stopRequested = false;
    orderActive = false;          // [SORTER] cancel order context on stop
    currentOrderId = "";
    DEBUG_PRINTLN("STOP Executed.");
  }

  if (homingRequested) {
    xSemaphoreTake(queueMutex, portMAX_DELAY);
    while (!commandQueue.empty()) commandQueue.pop();
    isExecuting = false;
    xSemaphoreGive(queueMutex);

    for (int i = 0; i < NUM_JOINTS; i++) {
      steppers[i].setCurrentPosition(0);
    }
    homingRequested = false;
    orderActive = false;          // [SORTER]
    currentOrderId = "";
    DEBUG_PRINTLN("Homing Complete.");
  }

  // Sequence Execution State Machine
  if (!isExecuting) {
    bool hasCommand = false;

    xSemaphoreTake(queueMutex, portMAX_DELAY);
    if (!commandQueue.empty()) {
      currentStep = commandQueue.front();
      commandQueue.pop();
      hasCommand = true;
    }
    xSemaphoreGive(queueMutex);

    // [SORTER FIX] order complete only after the queue stays drained —
    // debounced so a transient empty reading can never end an order early.
    static unsigned long orderIdleSince = 0;
    if (!hasCommand && orderActive) {
      if (orderIdleSince == 0) orderIdleSince = millis();
      else if (millis() - orderIdleSince >= 150) {
        lastOrderId = currentOrderId;
        currentOrderId = "";
        orderActive = false;
        orderIdleSince = 0;
        DEBUG_PRINT("Order complete: "); DEBUG_PRINTLN(lastOrderId);
      }
    } else {
      orderIdleSince = 0;
    }

    if (hasCommand) {
      if (currentStep.type == SET_SPEED) {
        currentSequenceSpeed = currentStep.param;
        if (currentSequenceSpeed > UNIVERSAL_MAX_SPEED_DEG_PER_SEC) currentSequenceSpeed = UNIVERSAL_MAX_SPEED_DEG_PER_SEC;
        if (currentSequenceSpeed < 1.0) currentSequenceSpeed = 1.0;
      } else if (currentStep.type == SET_ACCEL) {
        currentSequenceAccel = currentStep.param;
        if (currentSequenceAccel > UNIVERSAL_ACCEL_DEG_PER_SEC_SQ) currentSequenceAccel = UNIVERSAL_ACCEL_DEG_PER_SEC_SQ;
        if (currentSequenceAccel < 1.0) currentSequenceAccel = 1.0;
      } else if (currentStep.type == MOVE) {
        if (currentStep.toolheadCmd == 'C' || currentStep.toolheadCmd == 'c') {
          int pulse = map(85, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
          if (!servoActive) {
            toolheadServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
            servoActive = true;
          }
          toolheadServo.writeMicroseconds(pulse);
          servoActionTimer = millis();
          DEBUG_PRINTLN("Executing MOVE: Toolhead Closed.");
        } else if (currentStep.toolheadCmd == 'O' || currentStep.toolheadCmd == 'o') {
          int pulse = map(0, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
          if (!servoActive) {
            toolheadServo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
            servoActive = true;
          }
          toolheadServo.writeMicroseconds(pulse);
          servoActionTimer = millis();
          DEBUG_PRINTLN("Executing MOVE: Toolhead Opened.");
        } else {
          DEBUG_PRINTLN("Executing MOVE.");
        }

        float deltaAngles[NUM_JOINTS];
        float maxDeltaAngle = 0.0;
        for (int i = 0; i < NUM_JOINTS; i++) {
            float currentAngle = (float)steppers[i].currentPosition() / stepsPerDegree[i];
            deltaAngles[i] = abs(currentStep.angles[i] - currentAngle);
            if (deltaAngles[i] > maxDeltaAngle) {
                maxDeltaAngle = deltaAngles[i];
            }
        }

        for (int i = 0; i < NUM_JOINTS; i++) {
          long targetSteps = round(currentStep.angles[i] * stepsPerDegree[i]);

          if (maxDeltaAngle > 0.0) {
            float ratio = deltaAngles[i] / maxDeltaAngle;
            float scaledSpeed = max((currentSequenceSpeed * stepsPerDegree[i]) * ratio, 1.0f);
            float scaledAccel = max((currentSequenceAccel * stepsPerDegree[i]) * ratio, MIN_STEP_ACCEL);
            scaledSpeed = min(scaledSpeed, MAX_STEP_RATE_PER_JOINT[i]);   // [SORTER FIX v2]

            steppers[i].setMaxSpeed(scaledSpeed);
            steppers[i].setAcceleration(scaledAccel);
          }
          steppers[i].moveTo(targetSteps);
        }

        isExecuting = true;
      } else if (currentStep.type == DELAY) {
        delayStartTime = millis();
        isExecuting = true;
        DEBUG_PRINT("Executing DELAY: "); DEBUG_PRINT(currentStep.delayMs); DEBUG_PRINTLN(" ms");
      }
    }
  }

  // Command Completion Checker
  if (isExecuting) {
    if (currentStep.type == MOVE) {
      bool allDone = true;
      for (int i = 0; i < NUM_JOINTS; i++) {
        if (steppers[i].distanceToGo() != 0) {
          allDone = false;
          break;
        }
      }
      if (allDone) {
        isExecuting = false;
      }
    } else if (currentStep.type == DELAY) {
      if (millis() - delayStartTime >= currentStep.delayMs) {
        isExecuting = false;
      }
    }
  }

  // Step all motors UNCONDITIONALLY
  steppers[0].run();
  steppers[1].run();
  steppers[2].run();
  steppers[3].run();
  steppers[4].run();
  steppers[5].run();

  // --- Servo Limit Safety Locking ---
  if (servoActive && (millis() - servoActionTimer > 250)) {
    toolheadServo.detach();
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    servoActive = false;
  }
}
