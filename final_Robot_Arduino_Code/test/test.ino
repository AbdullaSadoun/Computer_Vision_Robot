/*
Here lives the test code for the robot arduino connections, 
*/

/*
Here are the details about the arduino connections:
- Arduino Nano
- Hc-05 Bluetooth Module
- R Continuous Servo
- L Continuous Servo
- Laser LED
- Laser turret positional Servo  at D11
*/

/*
Notes/Todo
Calibrate the Turret based on limits, code should start at 45
The code for the HC05 is 1234 

*/

#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

/*
  =========================
  PIN CONFIG (edit as needed)
  =========================
  HC-05:
    - HC05 TX -> Arduino BT_RX (PIN_BT_RX)
    - HC05 RX <- Arduino BT_TX (PIN_BT_TX) THROUGH DIVIDER
*/
static const uint8_t PIN_BT_RX   = 2;   // Arduino receives from HC-05 TX
static const uint8_t PIN_BT_TX   = 3;   // Arduino transmits to HC-05 RX (use divider)

static const uint8_t PIN_SERVO_L = 9;   // Left continuous servo signal
static const uint8_t PIN_SERVO_R = 10;  // Right continuous servo signal
static const uint8_t PIN_TURRET  = 11;  // Turret positional servo signal at D11

static const uint8_t PIN_LASER   = 6;   // Laser transistor gate/base signal

/*
  =========================
  SERVO TUNING
  =========================
  For most continuous servos:
    1500us ~ stop
    1000us ~ full speed one direction
    2000us ~ full speed other direction
*/
static const int SERVO_NEUTRAL_US = 1500;
static const int SERVO_SPAN_US    = 700;   // maps -100..100 -> +/- 400us (safe default)
static const bool LEFT_INVERT  = false;    // flip if left wheel goes opposite
static const bool RIGHT_INVERT = true;     // flip if right wheel goes opposite (common)

static const bool BATTERY_POWERED = true; // true if battery powered, false if USB powered
// Continuous servos usually need per-wheel neutral trim to fully stop.
static const int SERVO_L_NEUTRAL_US = SERVO_NEUTRAL_US + (BATTERY_POWERED ? -27 : -24);
static const int SERVO_R_NEUTRAL_US = SERVO_NEUTRAL_US + (BATTERY_POWERED ? -55 : -50);

/*
  =========================
  TURRET LIMITS
  =========================
  You can tighten these to avoid mechanical binding.
*/
static const uint8_t TURRET_MIN_DEG = 0;
static const uint8_t TURRET_MAX_DEG = 180;

/*
  =========================
  LASER SETTINGS
  =========================
*/
static const bool LASER_ACTIVE_HIGH = true;
static const uint16_t LASER_PULSE_MS = 250;  // pulse length
static const uint16_t CMD_TIMEOUT_MS = 250;  // failsafe stop if no valid cmd

/*
  =========================
  PACKET FORMAT
  =========================
  [0]=0xAA [1]=0x55
  [2]=seq
  [3]=flags
  [4]=left  (int8 -100..100)
  [5]=right (int8 -100..100)
  [6]=turret_deg (0..180)
  [7]=laser (0/1)
  [8]=reserved
  [9]=chk = XOR(bytes 2..8)
*/
static const uint8_t H1 = 0xAA;
static const uint8_t H2 = 0x55;

struct Cmd {
  uint8_t seq;
  uint8_t flags;
  int8_t  left;
  int8_t  right;
  uint8_t turret_deg;
  uint8_t laser;
  uint8_t reserved;
  uint8_t chk;
};

/*
  =========================
  FLAGS (bit positions)
  =========================
  bit0: MODE_ATTACK
  bit1: MODE_DEFENSE
  bit2: ESTOP
  bit3: FIRE_ARMED
*/
static const uint8_t FLAG_MODE_ATTACK = (1 << 0);
static const uint8_t FLAG_MODE_DEF    = (1 << 1);
static const uint8_t FLAG_ESTOP       = (1 << 2);
static const uint8_t FLAG_FIRE_ARMED  = (1 << 3);

/*
  =========================
  TEST MODE SELECT
  =========================
  You can run one test automatically at boot by setting TEST_MODE.

  0 = normal (Bluetooth packets control robot)
  1 = test drive
  2 = test turret
  3 = test laser
  4 = test checksum + parser (offline)
  5 = test bluetooth echo/ack
  6 = test WASD over Bluetooth packet simulation
*/
static const uint8_t TEST_MODE = 0;
static const int8_t WASD_DRIVE_SPEED = 70;
// Hold last WASD command briefly to avoid stop/go jitter between keystrokes. (between120 & 250) lower is more responsive stop higher is smoother but more coasting
static const uint16_t MODE6_CMD_HOLD_MS = 175;

/*
  =========================
  GLOBALS
  =========================
*/
SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);
Servo servoL, servoR, turret;

static uint32_t last_cmd_ms = 0;
static Cmd last_cmd = {0};

/*
  =========================
  UTIL / CORE FUNCTIONS
  =========================
*/

// --- (1) Checksum ---
uint8_t checksum_xor(const Cmd &c) {
  uint8_t x = 0;
  x ^= c.seq;
  x ^= c.flags;
  x ^= (uint8_t)c.left;
  x ^= (uint8_t)c.right;
  x ^= c.turret_deg;
  x ^= c.laser;
  x ^= c.reserved;
  return x;
}

// --- (2) Map wheel command (-100..100) to servo microseconds ---
int cmdToMicroseconds(int8_t cmd, bool invert, int neutralUs) {
  int v = (int)cmd;
  if (invert) v = -v;
  v = constrain(v, -100, 100);
  // linear map: -100..100 -> (neutral - span) .. (neutral + span)
  return neutralUs + (v * SERVO_SPAN_US) / 100;
}

// --- (3) Wheel drive setter ---
void setDrive(int8_t leftCmd, int8_t rightCmd) {
  int usL = cmdToMicroseconds(leftCmd, LEFT_INVERT, SERVO_L_NEUTRAL_US);
  int usR = cmdToMicroseconds(rightCmd, RIGHT_INVERT, SERVO_R_NEUTRAL_US);

  servoL.writeMicroseconds(usL);
  servoR.writeMicroseconds(usR);
}

// --- (4) Stop motors ---
void stopMotors() {
  servoL.writeMicroseconds(SERVO_L_NEUTRAL_US);
  servoR.writeMicroseconds(SERVO_R_NEUTRAL_US);
}

// --- (5) Turret setter ---
void setTurretAngle(uint8_t deg) {
  deg = constrain(deg, TURRET_MIN_DEG, TURRET_MAX_DEG);
  turret.write(deg);
}

// --- (6) Laser on/off ---
void setLaser(bool on) {
  if (LASER_ACTIVE_HIGH) digitalWrite(PIN_LASER, on ? HIGH : LOW);
  else                  digitalWrite(PIN_LASER, on ? LOW : HIGH);
}

// --- (7) Laser pulse fire (one-shot gating handled elsewhere) ---
void fireLaserPulse(uint16_t ms = LASER_PULSE_MS) {
  setLaser(true);
  delay(ms);
  setLaser(false);
}

// --- (8) Send ACK back over BT (simple) ---
void sendAck(uint8_t seq, uint8_t statusFlags) {
  // Simple ACK frame: 0xCC 0x33 seq status shot chk
  // 'shot' kept in frame for backward-compat; always 0 now (no Arduino-side latch).
  const uint8_t A1 = 0xCC, A2 = 0x33;
  uint8_t shot = 0;
  uint8_t chk = (uint8_t)(seq ^ statusFlags ^ shot);

  bt.write(A1);
  bt.write(A2);
  bt.write(seq);
  bt.write(statusFlags);
  bt.write(shot);
  bt.write(chk);
}

// --- (9) Packet parser state machine ---
bool readCommand(Stream &s, Cmd &out) {
  enum State { WAIT_H1, WAIT_H2, READ_PAYLOAD };
  static State st = WAIT_H1;
  static uint8_t idx = 0;
  static uint8_t payload[8];

  while (s.available()) {
    uint8_t b = (uint8_t)s.read();

    if (st == WAIT_H1) {
      if (b == H1) st = WAIT_H2;
    }
    else if (st == WAIT_H2) {
      if (b == H2) {
        st = READ_PAYLOAD;
        idx = 0;
      } else {
        st = WAIT_H1;
      }
    }
    else { // READ_PAYLOAD
      payload[idx++] = b;
      if (idx >= 8) {
        // payload mapping
        out.seq       = payload[0];
        out.flags     = payload[1];
        out.left      = (int8_t)payload[2];
        out.right     = (int8_t)payload[3];
        out.turret_deg= payload[4];
        out.laser     = payload[5];
        out.reserved  = payload[6];
        out.chk       = payload[7];

        st = WAIT_H1;

        // validate checksum
        return (out.chk == checksum_xor(out));
      }
    }
  }
  return false;
}

// --- (10) Apply command (ESTOP, wheels, turret, laser gating) ---
void applyCommand(const Cmd &c) {
  // E-STOP
  if (c.flags & FLAG_ESTOP) {
    stopMotors();
    setLaser(false);
    return;
  }

  // Wheels
  setDrive(c.left, c.right);

  // Turret
  setTurretAngle(c.turret_deg);

  // Laser: on while PC has it armed + requested. No latch, non-blocking.
  // PC already gates the fire window per shot; watchdog forces-off on comms loss.
  bool armed = (c.flags & FLAG_FIRE_ARMED);
  setLaser(armed && c.laser == 1);
}

// --- (11) Failsafe watchdog ---
void safetyWatchdog() {
  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    stopMotors();
    setLaser(false);
    // turret holds last position
  }
}

/*
  =========================
  TEST UTILITIES
  =========================
*/

// --- Test (A): drive function ---
void test_setDrive() {
  Serial.println(F("\n[TEST] setDrive: forward, stop, reverse, spin, stop"));
  setDrive(40, 40); delay(1500);
  stopMotors();     delay(800);

  setDrive(-40, -40); delay(1500);
  stopMotors();       delay(800);

  setDrive(40, -40);  delay(1500); // spin in place
  stopMotors();       delay(800);

  Serial.println(F("[TEST] setDrive done."));
}

// --- Test (B): turret function ---
void test_setTurretAngle() {
  Serial.println(F("\n[TEST] setTurretAngle sweep 0->180->0"));
  for (int d = 0; d <= 180; d += 5) {
    setTurretAngle(d); delay(60); 
    }
  for (int d = 180; d >= 0; d -= 5) { 
    setTurretAngle(d); delay(60); 
    }
  Serial.println(F("[TEST] turret sweep done."));
}

// --- Test (C): laser functions ---
void test_fireLaserPulse() {
  Serial.println(F("\n[TEST] Laser pulse 3 times"));
  for (int i = 0; i < 3; i++) {
    Serial.print(F("Pulse ")); Serial.println(i+1);
    fireLaserPulse(200);
    delay(600);
  }
  Serial.println(F("[TEST] Laser test done."));
}

// --- Test (D): checksum function ---
void test_checksum_xor() {
  Serial.println(F("\n[TEST] checksum_xor known vector"));
  Cmd c;
  c.seq=0x10; c.flags=0x08; c.left=10; c.right=-20; c.turret_deg=90; c.laser=1; c.reserved=0;
  uint8_t chk = checksum_xor(c);
  Serial.print(F("Computed chk=0x")); Serial.println(chk, HEX);
  Serial.println(F("Now set c.chk = computed and verify compare in your parser test."));
}

// --- A tiny fake Stream for parser testing ---
class ByteQueueStream : public Stream {
public:
  ByteQueueStream(const uint8_t *data, size_t len) : _data(data), _len(len), _i(0) {}
  int available() override { return (int)(_len - _i); }
  int read() override { return (_i < _len) ? _data[_i++] : -1; }
  int peek() override { return (_i < _len) ? _data[_i] : -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 0; } // not needed
private:
  const uint8_t *_data;
  size_t _len;
  size_t _i;
};

// --- Test (E): parser function (offline, deterministic) ---
void test_readCommand_parser() {
  Serial.println(F("\n[TEST] readCommand: build a packet and parse it"));

  Cmd c;
  c.seq=7; c.flags=FLAG_FIRE_ARMED; c.left=25; c.right=-25; c.turret_deg=123; c.laser=0; c.reserved=0;
  c.chk = checksum_xor(c);

  uint8_t pkt[10];
  pkt[0]=H1; pkt[1]=H2;
  pkt[2]=c.seq;
  pkt[3]=c.flags;
  pkt[4]=(uint8_t)c.left;
  pkt[5]=(uint8_t)c.right;
  pkt[6]=c.turret_deg;
  pkt[7]=c.laser;
  pkt[8]=c.reserved;
  pkt[9]=c.chk;

  ByteQueueStream fake(pkt, sizeof(pkt));
  Cmd out;
  bool ok = readCommand(fake, out);

  Serial.print(F("Parse ok? ")); Serial.println(ok ? F("YES") : F("NO"));
  if (ok) {
    Serial.print(F("seq=")); Serial.println(out.seq);
    Serial.print(F("flags=0x")); Serial.println(out.flags, HEX);
    Serial.print(F("left=")); Serial.println(out.left);
    Serial.print(F("right=")); Serial.println(out.right);
    Serial.print(F("turret=")); Serial.println(out.turret_deg);
    Serial.print(F("laser=")); Serial.println(out.laser);
  }
  Serial.println(F("[TEST] parser test done."));
}

// --- Test (F): bluetooth link (echo + ack) ---
void test_bluetooth_echo_ack() {
  Serial.println(F("\n[TEST] Bluetooth: type into Serial Monitor; forwarded to BT, and BT echoed to Serial"));
  Serial.println(F("Also sends a test ACK every 2 seconds over BT."));
  uint32_t t0 = millis();
  while (true) {
    if (Serial.available()) bt.write((uint8_t)Serial.read());
    if (bt.available()) Serial.write((uint8_t)bt.read());

    if (millis() - t0 > 2000) {
      t0 = millis();
      sendAck(0x42, 0x00);
      Serial.println(F("[TEST] sent ACK frame over BT."));
    }
  }
}

// --- Test (G): WASD over BT -> packet simulation path ---
bool mapWasdKeyToCmd(char key, uint8_t seq, Cmd &out) {
  out.seq = seq;
  out.flags = 0;
  out.left = 0;
  out.right = 0;
  out.turret_deg = 90;
  out.laser = 0;
  out.reserved = 0;

  switch (key) {
    case 'w': case 'W':
      out.left = WASD_DRIVE_SPEED;
      out.right = WASD_DRIVE_SPEED;
      break;
    case 's': case 'S':
      out.left = -WASD_DRIVE_SPEED;
      out.right = -WASD_DRIVE_SPEED;
      break;
    case 'a': case 'A':
      out.left = -WASD_DRIVE_SPEED;
      out.right = WASD_DRIVE_SPEED;
      break;
    case 'd': case 'D':
      out.left = WASD_DRIVE_SPEED;
      out.right = -WASD_DRIVE_SPEED;
      break;
    case 'x': case 'X':
    case ' ':
      out.left = 0;
      out.right = 0;
      break;
    default:
      return false;
  }

  out.chk = checksum_xor(out);
  return true;
}

void serializeCommandPacket(const Cmd &c, uint8_t outPacket[10]) {
  outPacket[0] = H1;
  outPacket[1] = H2;
  outPacket[2] = c.seq;
  outPacket[3] = c.flags;
  outPacket[4] = (uint8_t)c.left;
  outPacket[5] = (uint8_t)c.right;
  outPacket[6] = c.turret_deg;
  outPacket[7] = c.laser;
  outPacket[8] = c.reserved;
  outPacket[9] = c.chk;
}

void mode6_wasdOverBluetoothLoop() {
  static uint8_t seq = 0;
  static uint32_t last_mode6_valid_ms = 0;
  bool processedValidCommand = false;
  while (bt.available()) {
    char key = (char)bt.read();
    Cmd mapped;
    if (!mapWasdKeyToCmd(key, seq++, mapped)) continue;

    uint8_t pkt[10];
    serializeCommandPacket(mapped, pkt);

    ByteQueueStream frame(pkt, sizeof(pkt));
    Cmd parsed;
    if (readCommand(frame, parsed)) {
      last_cmd = parsed;
      last_cmd_ms = millis();
      last_mode6_valid_ms = last_cmd_ms;
      applyCommand(parsed);

      uint8_t status = 0;
      sendAck(parsed.seq, status);
      processedValidCommand = true;
    }
  }

  // In mode 6, keep the last valid command briefly to smooth key-repeat gaps.
  if (!processedValidCommand && (millis() - last_mode6_valid_ms > MODE6_CMD_HOLD_MS)) {
    stopMotors();
    setLaser(false);
  }

  safetyWatchdog();
}

/*
  =========================
  SERIAL COMMAND MENU (Bring-up)
  =========================
  Use this to test each component without PC strategy running.
*/
void printHelp() {
  Serial.println(F("\n=== Bring-up Menu ==="));
  Serial.println(F("h                : help"));
  Serial.println(F("d L R            : drive left/right (-100..100)"));
  Serial.println(F("t DEG            : turret angle (0..180)"));
  Serial.println(F("l 0|1            : laser off/on (no one-shot latch)"));
  Serial.println(F("f                : fire laser pulse (USB bench-test only)"));
  Serial.println(F("s                : stop motors + laser off"));
  Serial.println(F("tp               : test parser"));
  Serial.println(F("td               : test drive"));
  Serial.println(F("tt               : test turret"));
  Serial.println(F("tl               : test laser"));
  Serial.println(F("=====================\n"));
}

// Simple token parser
bool readIntFromSerial(int &out) {
  while (Serial.available() && isspace(Serial.peek())) Serial.read();
  if (!Serial.available()) return false;
  out = Serial.parseInt();
  return true;
}

void bringupMenuLoop() {
  if (!Serial.available()) return;

  char cmd = (char)Serial.read();
  if (cmd == '\n' || cmd == '\r') return;

  if (cmd == 'h') { printHelp(); }
  else if (cmd == 'd') {
    int L, R;
    if (readIntFromSerial(L) && readIntFromSerial(R)) {
      setDrive((int8_t)constrain(L, -100, 100), (int8_t)constrain(R, -100, 100));
      Serial.println(F("OK drive"));
    } else Serial.println(F("Usage: d L R"));
  }
  else if (cmd == 't') {
    int deg;
    if (readIntFromSerial(deg)) {
      setTurretAngle((uint8_t)constrain(deg, 0, 180));
      Serial.println(F("OK turret"));
    } else Serial.println(F("Usage: t DEG"));
  }
  else if (cmd == 'l') {
    int on;
    if (readIntFromSerial(on)) {
      setLaser(on != 0);
      Serial.println(F("OK laser"));
    } else Serial.println(F("Usage: l 0|1"));
  }
  else if (cmd == 'f') {
    fireLaserPulse();
    Serial.println(F("Fired pulse."));
  }
  else if (cmd == 's') {
    stopMotors();
    setLaser(false);
    Serial.println(F("STOP"));
  }
  else if (cmd == 't') { test_setTurretAngle(); }
  else if (cmd == 'p') { test_readCommand_parser(); }
  else if (cmd == 't') { test_setTurretAngle(); }
  else if (cmd == 'x') {
    test_checksum_xor();
    test_readCommand_parser();
    test_setTurretAngle();
    test_setDrive();
    test_fireLaserPulse();
  }
  // multi-letter commands:
  else {
    // allow td/tt/tl/tp typed quickly (read remaining chars)
    if (cmd == 't') return;
  }
}

/*
  =========================
  SETUP + LOOP
  =========================
*/
void setup() {
  pinMode(PIN_LASER, OUTPUT);
  setLaser(false);

  Serial.begin(115200);
  bt.begin(9600); // common HC-05 default in data mode

  servoL.attach(PIN_SERVO_L);
  servoR.attach(PIN_SERVO_R);
  turret.attach(PIN_TURRET);

  stopMotors();
  setTurretAngle(90);

  Serial.println(F("Nano Robot Controller Booted."));
  printHelp();

  // Auto tests
  if (TEST_MODE == 1) test_setDrive();
  if (TEST_MODE == 2) test_setTurretAngle();
  if (TEST_MODE == 3) test_fireLaserPulse();
  if (TEST_MODE == 4) { test_checksum_xor(); test_readCommand_parser(); }
  if (TEST_MODE == 5) test_bluetooth_echo_ack();
  if (TEST_MODE == 6) {
    Serial.println(F("[TEST] Mode 6 active: send W/A/S/D over Bluetooth (HC-05)."));
    Serial.println(F("[TEST] Send X or space to stop."));
    Serial.println(F("[TEST] Robot stays still until valid command is received."));
  }

  last_cmd_ms = millis();
}

void loop() {
  // 1) Bring-up menu available anytime (USB Serial)
  bringupMenuLoop();

  // 2) Normal mode: read commands from BT and apply
  Cmd c;
  if (TEST_MODE == 0) {
    if (readCommand(bt, c)) {
      last_cmd = c;
      last_cmd_ms = millis();
      applyCommand(c);

      // Send ack with minimal status (you can expand)
      uint8_t status = 0;
      sendAck(c.seq, status);
    }
    safetyWatchdog();
  }
  else if (TEST_MODE == 6) {
    mode6_wasdOverBluetoothLoop();
  }
}