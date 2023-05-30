HardwareSerial Serial2(0);

typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t motionX;
    int32_t motionY;
} mspSensorOpflowDataMessage_t;

typedef struct __attribute__((packed)) {
    uint8_t quality;    // [0;255]
    int32_t distanceMm; // Negative value for out of range
} mspSensorRangefinderDataMessage_t;


uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
}

void loop() {
  // Read the MSP header
  if (Serial2.available() >= 6) {
    char header[3];
    Serial2.readBytes(header, 3);
    if (header[0] == '$' && header[1] == 'X' && header[2] == '<') {
      uint8_t flag = Serial2.read(); // read flag
      uint16_t cmdId;
      Serial2.readBytes((char*)&cmdId, 2); // read command id
      uint16_t payloadSize;
      Serial2.readBytes((char*)&payloadSize, 2); // read payload size
      uint8_t checksumCalc = crc8_dvb_s2(0, flag);
      checksumCalc = crc8_dvb_s2(checksumCalc, cmdId & 0xFF);
      checksumCalc = crc8_dvb_s2(checksumCalc, (cmdId >> 8) & 0xFF);
      checksumCalc = crc8_dvb_s2(checksumCalc, payloadSize & 0xFF);
      checksumCalc = crc8_dvb_s2(checksumCalc, (payloadSize >> 8) & 0xFF);
      uint8_t data[payloadSize];
      if (payloadSize > 0 && Serial2.available() >= payloadSize) {
        Serial2.readBytes(data, payloadSize);
        for (int i = 0; i < payloadSize; i++) {
          checksumCalc = crc8_dvb_s2(checksumCalc, data[i]);
        }
      }

      if (Serial2.available() > 0) {
        uint8_t checksum = Serial2.read(); // read checksum
        if (checksum == checksumCalc) {
          if (cmdId == 7938 && payloadSize == sizeof(mspSensorOpflowDataMessage_t)) {
            mspSensorOpflowDataMessage_t* msg = reinterpret_cast<mspSensorOpflowDataMessage_t*>(data);
            Serial.print("Motion X: ");
            Serial.print(msg->motionX);
            Serial.print(", Motion Y: ");
            Serial.print(msg->motionY);
            Serial.print(", Quality: ");
            Serial.println(msg->quality);
        } else if (cmdId == 7937 && payloadSize == sizeof(mspSensorRangefinderDataMessage_t)) {
            mspSensorRangefinderDataMessage_t* msg = reinterpret_cast<mspSensorRangefinderDataMessage_t*>(data);
            Serial.print("Distance: ");
            Serial.print(msg->distanceMm);
            Serial.print(", Quality: ");
            Serial.println(msg->quality);
        }
        } else {
          Serial.println("Checksum mismatch, handle error...");
        }
      }
    }
  }
}