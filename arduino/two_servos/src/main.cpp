#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>

namespace {
constexpr int    DXL_DIR_PIN = 2;
constexpr unsigned long DXL_BAUD = 57600;
constexpr unsigned long PC_BAUD  = 115200;
constexpr float  DXL_PROTOCOL_VERSION = 2.0f;

auto& dxlSerial = Serial1;  
auto& pcSerial  = Serial;  

// scan IDs before starting
constexpr std::array<uint8_t, 3> CANDIDATE_IDS = {1, 2, 3};
constexpr int      ENCODER_RES = 12;                        // 12-bit absolute encoder
constexpr int32_t  POS_MAX     = (1L << ENCODER_RES) - 1;   // 4095
constexpr uint32_t PROFILE_VEL = 15;
constexpr unsigned long STATE_PERIOD_MS = 50;               // 20 Hz
constexpr uint32_t READ_TIMEOUT_MS = 10;

Dynamixel2Arduino dxl(dxlSerial, DXL_DIR_PIN);
using namespace ControlTableItem;

// Result of the startup scan: the servos that answered, in CANDIDATE_IDS order.
std::array<uint8_t, CANDIDATE_IDS.size()> connectedIds{};
size_t numConnected = 0;

char   rxBuf[64];
size_t rxLen = 0;

void pcPrintf(const char* fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  pcSerial.print(buf);
}

bool isConnected(int id) {
  const auto end = connectedIds.begin() + numConnected;
  return std::find(connectedIds.begin(), end, id) != end;
}

void sendConnected() {
  pcPrintf("C %u", static_cast<unsigned>(numConnected));
  for (size_t i = 0; i < numConnected; ++i) {
    pcPrintf(" %u", connectedIds[i]);
  }
  pcSerial.print('\n');
}

bool configureServo(uint8_t id) {
  if (!dxl.ping(id)) {
    pcPrintf("# ID %u not found\n", id);
    return false;
  }
  pcPrintf("# ID %u found, HW error %ld, voltage x0.1V %ld\n", id,
           static_cast<long>(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id)),
           static_cast<long>(dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, id)));

  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROFILE_VEL);
  dxl.torqueOn(id);
  return true;
}

void scanServos() {
  numConnected = 0;
  for (uint8_t id : CANDIDATE_IDS) {
    if (configureServo(id)) {
      connectedIds[numConnected++] = id;
    }
  }
  pcPrintf("# %u of %u servos connected\n", static_cast<unsigned>(numConnected),
           static_cast<unsigned>(CANDIDATE_IDS.size()));
  sendConnected();
}

void sendState() {
  pcPrintf("S %lu", millis());
  for (size_t i = 0; i < numConnected; ++i) {
    const uint8_t id = connectedIds[i];
    long pos = static_cast<long>(dxl.readControlTableItem(PRESENT_POSITION, id, READ_TIMEOUT_MS));
    if (dxl.getLastLibErrCode() != DXL_LIB_OK) {
      pcPrintf(" %u 0 0 0 -1", id);    // no reply: skip the other reads to keep the stream on time
      continue;
    }
    long vel    = static_cast<long>(dxl.readControlTableItem(PRESENT_VELOCITY, id, READ_TIMEOUT_MS));
    long moving = static_cast<long>(dxl.readControlTableItem(MOVING, id, READ_TIMEOUT_MS));
    long hwerr  = static_cast<long>(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id, READ_TIMEOUT_MS));
    pcPrintf(" %u %ld %ld %ld %ld", id, pos, vel, moving, hwerr);
  }
  pcSerial.print('\n');
}

void handleCommand(const char* line) {
  char cmd   = 0;
  int  id    = 0;
  long value = 0;
  const int fields = sscanf(line, " %c %d %ld", &cmd, &id, &value);
  if (fields == 1 && cmd == 'C') {
    sendConnected();
    return;
  }
  if (fields != 3) {
    pcPrintf("E bad command: %s\n", line);
    return;
  }
  if (!isConnected(id)) {
    pcPrintf("E id %d not connected\n", id);
    return;
  }

  bool ok = true;
  switch (cmd) {
    case 'G': {
      int32_t goal = std::clamp<int32_t>(value, 0, POS_MAX);
      dxl.torqueOn(id);                // in case servo power dropped and came back
      ok = dxl.setGoalPosition(id, goal);
      break;
    }
    case 'T':
      ok = value ? dxl.torqueOn(id) : dxl.torqueOff(id);
      break;
    default:
      pcPrintf("E bad command: %s\n", line);
      return;
  }
  if (!ok) {
    pcPrintf("E no reply from id %d\n", id);
  }
}

// Collect incoming characters without blocking; run each complete line.
void pollCommands() {
  while (pcSerial.available() > 0) {
    char c = static_cast<char>(pcSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleCommand(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                       // line too long: drop it
      pcSerial.print("E line too long\n");
    }
  }
}

}  // namespace

void setup() {
  pcSerial.begin(PC_BAUD);
  delay(1000);                         // let the USB bridge come up
  pcSerial.print("# === Servo stream ===\n");

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  scanServos();
  pcPrintf("# streaming state every %lu ms\n", STATE_PERIOD_MS);
}

void loop() {
  pollCommands();

  static unsigned long lastState = millis();
  if (millis() - lastState >= STATE_PERIOD_MS) {
    lastState += STATE_PERIOD_MS;
    if (millis() - lastState >= STATE_PERIOD_MS) {
      lastState = millis();            
    }
    sendState();
  }
}
