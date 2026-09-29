#include <Arduino.h>

#include <cstring>

namespace {
constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kBusBaud = 1000000;
constexpr int kBusRxPin = 16;
constexpr int kBusTxPin = 17;
constexpr uint8_t kRightId = 1;
constexpr uint8_t kLeftId = 2;
constexpr uint8_t kC3Id = 3;
constexpr uint8_t kInstPing = 0x01;
constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;
constexpr uint8_t kInstAscii = 0xA0;
constexpr uint8_t kPresentPositionAddress = 56;
constexpr uint8_t kAccelerationAddress = 41;
constexpr uint32_t kResponseTimeoutMs = 30;

constexpr int16_t kRightHome = 2061;
constexpr int16_t kRightExtend = 2044;
constexpr int16_t kLeftHome = 2026;
constexpr int16_t kLeftExtend = 2044;
constexpr uint16_t kMoveSpeed = 150;
constexpr uint8_t kMoveAcceleration = 15;

struct Packet {
  // The C3 protocol permits up to 253 response parameters.
  uint8_t bytes[259];
  size_t size;
};

uint8_t checksum(const uint8_t* data, size_t first, size_t last) {
  uint8_t sum = 0;
  for (size_t i = first; i < last; ++i) sum += data[i];
  return static_cast<uint8_t>(~sum);
}

void printHex(const char* label, const uint8_t* data, size_t size) {
  Serial.print(label);
  for (size_t i = 0; i < size; ++i) Serial.printf(" %02X", data[i]);
  Serial.println();
}

void clearBusInput() {
  while (Serial2.read() >= 0) {}
}

bool samePacket(const Packet& packet, const uint8_t* sent, size_t sentSize) {
  if (packet.size != sentSize) return false;
  return memcmp(packet.bytes, sent, sentSize) == 0;
}

bool readPacket(Packet& packet, uint32_t deadlineMs) {
  uint8_t previous = 0;
  bool havePrevious = false;
  while (static_cast<int32_t>(deadlineMs - millis()) >= 0) {
    const int value = Serial2.read();
    if (value < 0) {
      delay(1);
      continue;
    }
    const uint8_t current = static_cast<uint8_t>(value);
    if (havePrevious && previous == 0xFF && current == 0xFF) {
      packet.bytes[0] = 0xFF;
      packet.bytes[1] = 0xFF;
      packet.size = 2;
      break;
    }
    previous = current;
    havePrevious = true;
  }
  if (packet.size != 2) return false;

  while (packet.size < 4 && static_cast<int32_t>(deadlineMs - millis()) >= 0) {
    const int value = Serial2.read();
    if (value >= 0) packet.bytes[packet.size++] = static_cast<uint8_t>(value);
    else delay(1);
  }
  if (packet.size != 4) return false;
  const size_t totalSize = static_cast<size_t>(packet.bytes[3]) + 4;
  if (totalSize > sizeof(packet.bytes) || totalSize < 6) return false;
  while (packet.size < totalSize &&
         static_cast<int32_t>(deadlineMs - millis()) >= 0) {
    const int value = Serial2.read();
    if (value >= 0) packet.bytes[packet.size++] = static_cast<uint8_t>(value);
    else delay(1);
  }
  return packet.size == totalSize;
}

bool transact(const uint8_t* request, size_t requestSize, uint8_t expectedId,
              Packet& response) {
  clearBusInput();
  printHex("TX:", request, requestSize);
  Serial2.write(request, requestSize);
  Serial2.flush();

  const uint32_t deadline = millis() + kResponseTimeoutMs;
  while (static_cast<int32_t>(deadline - millis()) >= 0) {
    Packet candidate = {};
    if (!readPacket(candidate, deadline)) break;
    printHex("RX:", candidate.bytes, candidate.size);
    if (samePacket(candidate, request, requestSize)) {
      Serial.println("RX classification: TX echo (ignored)");
      continue;
    }
    if (candidate.bytes[2] != expectedId) {
      Serial.printf("RX classification: other ID %u (ignored)\n",
                    candidate.bytes[2]);
      continue;
    }
    const uint8_t actual = candidate.bytes[candidate.size - 1];
    const uint8_t expected = checksum(candidate.bytes, 2, candidate.size - 1);
    if (actual != expected) {
      Serial.printf("RX classification: bad checksum (got=%02X expected=%02X)\n",
                    actual, expected);
      continue;
    }
    response = candidate;
    Serial.printf("RX classification: valid status (error=%02X)\n",
                  response.bytes[4]);
    return true;
  }
  Serial.printf("TIMEOUT: ID %u, no valid non-echo response in %lu ms\n",
                expectedId, static_cast<unsigned long>(kResponseTimeoutMs));
  return false;
}

bool pingServo(uint8_t id) {
  uint8_t request[] = {0xFF, 0xFF, id, 2, kInstPing, 0};
  request[5] = checksum(request, 2, 5);
  Packet response = {};
  Serial.printf("PING ID %u (no motion command)\n", id);
  bool ok = transact(request, sizeof(request), id, response);
  if (ok && (response.size != 6 || response.bytes[3] != 2)) {
    Serial.printf("RESULT: ID %u invalid Ping response length=%u\n", id,
                  response.bytes[3]);
    ok = false;
  }
  if (ok && response.bytes[4] != 0) {
    Serial.printf("RESULT: ID %u Ping error=%02X\n", id, response.bytes[4]);
    ok = false;
  }
  Serial.printf("RESULT: ID %u %s\n", id, ok ? "RESPONDED" : "NO RESPONSE");
  return ok;
}

void sendC3AsciiCommand(const char* command,
                        const char* expectedResponse = nullptr) {
  const size_t parameterCount = strlen(command);
  if (parameterCount == 0 || parameterCount > 253) {
    Serial.println("RESULT: invalid local C3 command length; nothing sent");
    return;
  }

  uint8_t request[259] = {0xFF, 0xFF, kC3Id,
                          static_cast<uint8_t>(parameterCount + 2), kInstAscii};
  memcpy(request + 5, command, parameterCount);
  const size_t requestSize = parameterCount + 6;
  request[requestSize - 1] = checksum(request, 2, requestSize - 1);

  Packet response = {};
  Serial.printf("C3 ASCII ID %u: %s\n", kC3Id, command);
  if (!transact(request, requestSize, kC3Id, response)) return;

  const uint8_t length = response.bytes[3];
  const uint8_t error = response.bytes[4];
  Serial.printf("C3 response: ID=%u length=%u error=%02X checksum=OK\n",
                response.bytes[2], length, error);
  if (error != 0) {
    Serial.println("RESULT: C3 returned an error; response rejected");
    return;
  }

  const size_t responseParameterCount = static_cast<size_t>(length) - 2;
  for (size_t i = 0; i < responseParameterCount; ++i) {
    const uint8_t value = response.bytes[5 + i];
    if (value < 0x20 || value > 0x7E) {
      Serial.printf("RESULT: non-ASCII response parameter at offset %u: %02X\n",
                    static_cast<unsigned>(i), value);
      return;
    }
  }
  Serial.print("C3 ASCII response: ");
  Serial.write(response.bytes + 5, responseParameterCount);
  Serial.println();
  if (expectedResponse != nullptr) {
    const size_t expectedResponseLength = strlen(expectedResponse);
    if (responseParameterCount != expectedResponseLength ||
        memcmp(response.bytes + 5, expectedResponse,
               expectedResponseLength) != 0) {
      Serial.printf("RESULT: unexpected C3 response; expected: %s\n",
                    expectedResponse);
      return;
    }
  }
  Serial.println(
      "RESULT: valid C3 response (command outcome is the response text above)");
}

void handleC3Tilt(const String& command) {
  constexpr char kPrefix[] = "c3tilt ";
  constexpr uint16_t kMinimumPulseUs = 500;
  constexpr uint16_t kMaximumPulseUs = 2400;

  if (!command.startsWith(kPrefix)) {
    Serial.println(
        "ERROR: usage: c3tilt <500-2400>; nothing was transmitted");
    return;
  }

  const String pulseText = command.substring(sizeof(kPrefix) - 1);
  if (pulseText.isEmpty()) {
    Serial.println(
        "ERROR: usage: c3tilt <500-2400>; nothing was transmitted");
    return;
  }

  uint32_t pulseUs = 0;
  for (size_t i = 0; i < pulseText.length(); ++i) {
    const char value = pulseText.charAt(i);
    if (value < '0' || value > '9') {
      Serial.println(
          "ERROR: c3tilt requires digits only (no sign or extra characters); "
          "nothing was transmitted");
      return;
    }
    pulseUs = pulseUs * 10 + static_cast<uint32_t>(value - '0');
    if (pulseUs > kMaximumPulseUs) break;
  }
  if (pulseUs < kMinimumPulseUs || pulseUs > kMaximumPulseUs) {
    Serial.println(
        "ERROR: c3tilt pulse must be from 500 through 2400 us; nothing was "
        "transmitted");
    return;
  }

  char request[16];
  char expectedResponse[24];
  snprintf(request, sizeof(request), "TILT %lu",
           static_cast<unsigned long>(pulseUs));
  snprintf(expectedResponse, sizeof(expectedResponse), "OK TILT %lu us",
           static_cast<unsigned long>(pulseUs));
  sendC3AsciiCommand(request, expectedResponse);
}

void handleC3Fire(const String& command) {
  constexpr char kPrefix[] = "c3fire ";

  if (!command.startsWith(kPrefix)) {
    Serial.println("ERROR: usage: c3fire <1-9>; nothing was transmitted");
    return;
  }

  const String numberText = command.substring(sizeof(kPrefix) - 1);
  // Deliberately accept exactly one ASCII digit. This rejects missing values,
  // signs, whitespace, and suffixes before any packet can reach the bus.
  if (numberText.length() != 1 || numberText.charAt(0) < '1' ||
      numberText.charAt(0) > '9') {
    Serial.println(
        "ERROR: c3fire requires exactly one digit from 1 through 9 (no sign "
        "or extra characters); nothing was transmitted");
    return;
  }

  char request[] = "FIRE 0";
  request[5] = numberText.charAt(0);
  sendC3AsciiCommand(request);
  Serial.println(
      "NOTICE: the response reports protocol validity only; firing outcome is "
      "not inferred and FIRE will not be retried");
}

void handleC3Mute(const String& command) {
  if (command == "c3mute on") {
    sendC3AsciiCommand("MUTE ON", "OK MUTE ON");
    return;
  }
  if (command == "c3mute off") {
    sendC3AsciiCommand("MUTE OFF", "OK MUTE OFF");
    return;
  }

  // Match only the two complete commands above. In particular, do not trim
  // whitespace or accept suffixes, so malformed actuator input cannot reach
  // the bus.
  Serial.println(
      "ERROR: usage: c3mute on|off (no extra characters); nothing was "
      "transmitted");
}

void readPosition(uint8_t id) {
  uint8_t request[] = {0xFF, 0xFF, id, 4, kInstRead,
                       kPresentPositionAddress, 2, 0};
  request[7] = checksum(request, 2, 7);
  Packet response = {};
  Serial.printf("READ POSITION ID %u\n", id);
  if (!transact(request, sizeof(request), id, response)) return;
  if (response.size != 8 || response.bytes[3] != 4) {
    Serial.printf("RESULT: unexpected position response length=%u\n",
                  response.bytes[3]);
    return;
  }
  const uint16_t position =
      static_cast<uint16_t>(response.bytes[5]) |
      (static_cast<uint16_t>(response.bytes[6]) << 8);
  Serial.printf("RESULT: ID %u position=%u raw\n", id, position);
}

void moveServo(uint8_t id, int16_t position, const char* action) {
  // STS3215 write at ACC(41): ACC, position L/H, time L/H, speed L/H.
  uint8_t request[] = {
      0xFF, 0xFF, id, 10, kInstWrite, kAccelerationAddress,
      kMoveAcceleration, static_cast<uint8_t>(position & 0xFF),
      static_cast<uint8_t>((position >> 8) & 0xFF), 0, 0,
      static_cast<uint8_t>(kMoveSpeed & 0xFF),
      static_cast<uint8_t>((kMoveSpeed >> 8) & 0xFF), 0};
  request[13] = checksum(request, 2, 13);
  Serial.printf("MOVE: ID=%u action=%s target=%d speed=%u acc=%u\n", id,
                action, position, kMoveSpeed, kMoveAcceleration);
  clearBusInput();
  printHex("TX:", request, sizeof(request));
  Serial2.write(request, sizeof(request));
  Serial2.flush();
  Serial.println("RESULT: command sent once; no retry and no motion inferred");
}

void printHelp() {
  Serial.println("\n=== 1 Mbps BUS DIAGNOSTIC (wheel motors are never initialized) ===");
  Serial.println("help       : show this menu");
  Serial.println("ping1      : ping right STS3215 ID 1");
  Serial.println("ping2      : ping left STS3215 ID 2");
  Serial.println("ping3      : standard Ping to C3 ID 3");
  Serial.println("pingall    : ping ID 1, ID 2, and ID 3 once each");
  Serial.println("pos1/pos2  : read current STS3215 position");
  Serial.println("rhome/rext : move right ID 1 to HOME/small extension");
  Serial.println("lhome/lext : move left ID 2 to HOME/small extension");
  Serial.println("c3ping     : send C3 0xA0 ASCII command PING");
  Serial.println("c3help     : send C3 0xA0 ASCII command HELP");
  Serial.println("c3status   : send C3 0xA0 ASCII command STATUS");
  Serial.println("c3arm      : send C3 0xA0 ASCII command ARM once");
  Serial.println("c3fire <n> : send FIRE <n> once (n is one digit, 1-9)");
  Serial.println("c3disarm   : send C3 0xA0 ASCII command DISARM once");
  Serial.println("c3stop     : send C3 0xA0 ASCII command STOP once");
  Serial.println("c3tilt <us>: set C3 ID 3 barrel servo pulse (500-2400 us)");
  Serial.println("c3mute on  : mute FIRE automatic firing audio (RAM state only)");
  Serial.println("c3mute off : enable FIRE automatic firing audio (RAM state only)");
  Serial.println("MUTE does not suppress manually requested SOUND playback.");
  Serial.println("No automatic ARM, FIRE retry, or local ARM-state tracking is used.");
}

void handleCommand(String command) {
  // Serial monitors commonly terminate a line with CRLF. Remove only that
  // framing CR; do not trim user input because actuator arguments must reject
  // leading/trailing whitespace rather than silently normalizing it.
  if (command.endsWith("\r")) command.remove(command.length() - 1);
  command.toLowerCase();
  if (command.isEmpty()) return;
  Serial.printf("COMMAND: %s\n", command.c_str());
  if (command == "help" || command == "h" || command == "?") printHelp();
  else if (command == "ping1") pingServo(kRightId);
  else if (command == "ping2") pingServo(kLeftId);
  else if (command == "ping3") pingServo(kC3Id);
  else if (command == "pingall") {
    pingServo(kRightId);
    pingServo(kLeftId);
    pingServo(kC3Id);
  } else if (command == "pos1") readPosition(kRightId);
  else if (command == "pos2") readPosition(kLeftId);
  else if (command == "rhome") moveServo(kRightId, kRightHome, "HOME");
  else if (command == "rext") moveServo(kRightId, kRightExtend, "SMALL_EXTEND");
  else if (command == "lhome") moveServo(kLeftId, kLeftHome, "HOME");
  else if (command == "lext") moveServo(kLeftId, kLeftExtend, "SMALL_EXTEND");
  else if (command == "c3ping") sendC3AsciiCommand("PING");
  else if (command == "c3help") sendC3AsciiCommand("HELP");
  else if (command == "c3status") sendC3AsciiCommand("STATUS");
  else if (command == "c3arm") sendC3AsciiCommand("ARM");
  else if (command == "c3disarm") sendC3AsciiCommand("DISARM");
  else if (command == "c3stop") sendC3AsciiCommand("STOP");
  else if (command == "c3fire" || command.startsWith("c3fire"))
    handleC3Fire(command);
  else if (command == "c3tilt" || command.startsWith("c3tilt"))
    handleC3Tilt(command);
  else if (command == "c3mute" || command.startsWith("c3mute"))
    handleC3Mute(command);
  else Serial.println("Unknown command; enter 'help'. Nothing was transmitted.");
}
}  // namespace

void setup() {
  Serial.begin(kUsbBaud);
  Serial2.begin(kBusBaud, SERIAL_8N1, kBusRxPin, kBusTxPin);
  delay(200);
  Serial.println("Bus diagnostic ready: RX=GPIO16 TX=GPIO17 1000000 8N1");
  printHelp();
}

void loop() {
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}
