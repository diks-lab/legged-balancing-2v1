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
  uint8_t bytes[32];
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
  const bool ok = transact(request, sizeof(request), id, response);
  Serial.printf("RESULT: ID %u %s\n", id, ok ? "RESPONDED" : "NO RESPONSE");
  return ok;
}

void pingC3Unavailable() {
  Serial.printf("ID %u: NOT SENT - legged-balancing-2c3 packet and response "
                "format were not available for source verification.\n", kC3Id);
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

void printC3CommandsUnavailable() {
  Serial.println("ID 3 COMMAND: NOT SENT");
  Serial.println("Reason: current legged-balancing-2c3 command framing, checksum, "
                 "and response format have not been verified from source.");
}

void printHelp() {
  Serial.println("\n=== 1 Mbps BUS DIAGNOSTIC (wheel motors are never initialized) ===");
  Serial.println("help       : show this menu");
  Serial.println("ping1      : ping right STS3215 ID 1");
  Serial.println("ping2      : ping left STS3215 ID 2");
  Serial.println("ping3      : report ID 3 protocol unavailable; transmit nothing");
  Serial.println("pingall    : ping ID 1/2; report ID 3 unavailable");
  Serial.println("pos1/pos2  : read current STS3215 position");
  Serial.println("rhome/rext : move right ID 1 to HOME/small extension");
  Serial.println("lhome/lext : move left ID 2 to HOME/small extension");
  Serial.println("c3help/c3status/c3laser/c3tilt/c3sound");
  Serial.println("           : disabled until 2c3 source protocol is verified");
}

void handleCommand(String command) {
  command.trim();
  command.toLowerCase();
  if (command.isEmpty()) return;
  Serial.printf("COMMAND: %s\n", command.c_str());
  if (command == "help" || command == "h" || command == "?") printHelp();
  else if (command == "ping1") pingServo(kRightId);
  else if (command == "ping2") pingServo(kLeftId);
  else if (command == "ping3") pingC3Unavailable();
  else if (command == "pingall") {
    pingServo(kRightId);
    pingServo(kLeftId);
    pingC3Unavailable();
  } else if (command == "pos1") readPosition(kRightId);
  else if (command == "pos2") readPosition(kLeftId);
  else if (command == "rhome") moveServo(kRightId, kRightHome, "HOME");
  else if (command == "rext") moveServo(kRightId, kRightExtend, "SMALL_EXTEND");
  else if (command == "lhome") moveServo(kLeftId, kLeftHome, "HOME");
  else if (command == "lext") moveServo(kLeftId, kLeftExtend, "SMALL_EXTEND");
  else if (command.startsWith("c3")) printC3CommandsUnavailable();
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
