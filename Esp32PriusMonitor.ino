// === ESP32 Prius CAN Parser ===
// Monitors Prius Gen2 Battery ECU on 2 CAN buses

#include <ACAN_ESP32.h>
#include <mcp_can.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);

// === Battery 1 (ESP32 CAN) ===
float packVoltage1 = 0.0;
float packCurrent1 = 0.0;
int soc1 = 0;
int ccl1 = 0, cdl1 = 0;
int temp1a = 0, temp1b = 0;
uint16_t faultCode1 = 0;
uint8_t deltaSOC1 = 0, flags1 = 0;
uint16_t calibX1 = 0, calibY1 = 0, calibZ1 = 0;
float blockVoltages1[14] = {0};
uint8_t isoTPBuf1[64];
size_t isoTPLen1 = 0;

// === Battery 2 (MCP2515 CAN) ===
float packVoltage2 = 0.0;
float packCurrent2 = 0.0;
int soc2 = 0;
int ccl2 = 0, cdl2 = 0;
int temp2a = 0, temp2b = 0;
uint16_t faultCode2 = 0;
uint8_t deltaSOC2 = 0, flags2 = 0;
uint16_t calibX2 = 0, calibY2 = 0, calibZ2 = 0;
float blockVoltages2[14] = {0};
uint8_t isoTPBuf2[64];
size_t isoTPLen2 = 0;

MCP_CAN can2(15);
unsigned long lastTesterPing = 0;
const unsigned long testerPingInterval = 3000; // 3 sec
int pid = 1;

void setup() {
  Serial.begin(115200);
  WiFi.softAP("PriusMonitor", "hybridpower");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><head><meta http-equiv='refresh' content='2'></head><body><h1>Prius BMS Data</h1>";
    html += "<h2>Battery 1</h2>";
    html += "<p>Voltage: " + String(packVoltage1) + " V</p>";
    html += "<p>Current: " + String(packCurrent1) + " A</p>";
    html += "<p>SOC: " + String(soc1/2.0) + " %</p>";
    html += "<p>Delta SOC: " + String(deltaSOC1/2.0) + " %</p>";
    html += "<p>CCL/CDL: " + String(ccl1) + "/" + String(cdl1) + " A</p>";
    html += "<p>Temps: " + String(temp1a) + ", " + String(temp1b) + "</p>";
    html += "<p>Flags: 0x" + String(flags1, HEX) + "</p>";
    html += "<p>Blocks:</p><ul>";
    for (int i=0; i<14; i++) html += "<li>Block "+String(i+1)+": "+String(blockVoltages1[i])+" V</li>";
    html += "</ul>";

    html += "<h2>Battery 2</h2>";
    html += "<p>Voltage: " + String(packVoltage2) + " V</p>";
    html += "<p>Current: " + String(packCurrent2) + " A</p>";
    html += "<p>SOC: " + String(soc2/2.0) + " %</p>";
    html += "<p>Delta SOC: " + String(deltaSOC2/2.0) + " %</p>";
    html += "<p>CCL/CDL: " + String(ccl2) + "/" + String(cdl2) + " A</p>";
    html += "<p>Temps: " + String(temp2a) + ", " + String(temp2b) + "</p>";
    html += "<p>Flags: 0x" + String(flags2, HEX) + "</p>";
    html += "<p>Blocks:</p><ul>";
    for (int i=0; i<14; i++) html += "<li>Block "+String(i+1)+": "+String(blockVoltages2[i])+" V</li>";
    html += "</ul>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });
  server.begin();

  ACAN_ESP32_Settings settings1(500000);
  settings1.mRxPin = GPIO_NUM_16;
  settings1.mTxPin = GPIO_NUM_17;
  if (ACAN_ESP32::can.begin(settings1) == 0) {
    Serial.println("Internal CAN OK");
  } else {
    Serial.println("Internal CAN FAILED");
  }

  SPI.begin();
  while (CAN_OK != can2.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)) {
    Serial.println("MCP2515 init failed. Retrying...");
    delay(1000);
  }
  can2.setMode(MCP_NORMAL);
  Serial.println("MCP2515 CAN OK");
}

void loop() {
  CANMessage frame;
  if (ACAN_ESP32::can.receive(frame)) parseCAN(frame, 1);
  long unsigned int rxId; uint8_t len = 0, buf[8];
  if (can2.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
    CANMessage m; m.id = rxId; m.len = len; memcpy(m.data, buf, len);
    parseCAN(m, 2);
  }

  if (millis() - lastTesterPing >= testerPingInterval) {
    lastTesterPing = millis();
    uint8_t req1[] = {0x02, 0x21, 0xCE,0,0,0,0,0};
    uint8_t req2[] = {0x02, 0x21, 0xCE,0,0,0,0,0};
    uint8_t req3[] = {0x02, 0x21, 0xCE,0,0,0,0,0};
    uint8_t req4[] = {0x02, 0x21, 0xCE,0,0,0,0,0};
    CANMessage tx; tx.id = 0x7E3; tx.len = 8;

    if (pid==1) memcpy(tx.data, req1, 8);
    if (pid==2) memcpy(tx.data, req2, 8);
    if (pid==3) memcpy(tx.data, req3, 8);
    if (pid==4) memcpy(tx.data, req4, 8);

    ACAN_ESP32::can.tryToSend(tx);
    can2.sendMsgBuf(0x7E3, 0, 8, tx.data);
    pid++; if (pid>4) pid=1;
  }
}

void parsePIDMap(uint8_t* data, size_t len) {
  if (len < 4) return;
  uint8_t base = data[2];
  Serial.printf("[PIDMAP] Base 0x%02X\n", base);
  for (int i=0; i<4; i++) {
    uint8_t b = data[3+i];
    for (int bit=7; bit>=0; bit--) {
      if (b & (1<<bit)) {
        uint8_t pid = base + (i*8 + (7-bit)) + 1;
        Serial.printf("  -> PID: 0x%02X\n", pid);
      }
    }
  }
}

/*void decodeBlocks(uint8_t* data, size_t len, int bus) {
  float* blocks = (bus==1) ? blockVoltages1 : blockVoltages2;

  const int headerLen = 3; // typical for UDS 62 PID

  //if (data[1] == 0xCE) {
  for (int i = 0; i < 14; i++) {
    int pos = headerLen + i*2;
    if (pos+1 >= len) break;
    uint16_t raw = (data[pos] << 8) | data[pos+1];
    blocks[i] = raw * 0.000415; // or your final scale
  }

  Serial.printf("[BUS%d] data:", bus);
  for (int i = 0; i < len; i++) Serial.printf(" %#02x", data[i]);
  Serial.println();

  Serial.printf("[BUS%d] Blocks:", bus);
  for (int i = 0; i < 14; i++) Serial.printf(" %.2f", blocks[i]);
  Serial.println();
  //}
}*/

void decodeBlocks(uint8_t* data, size_t len, int bus) {
  float* blocks = (bus == 1) ? blockVoltages1 : blockVoltages2;

  // We expect at least 2 bytes of header + 14 * 2 = 30 bytes of data
  if (len < 2 + 14 * 2) {
    Serial.printf("[BUS%d] decodeBlocks() called with insufficient data!\n", bus);
    return;
  }

  Serial.printf("[BUS%d] data:", bus);
  for (size_t i = 0; i < len; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();

  for (int i = 0; i < 14; i++) {
    uint8_t D = data[2 + i * 2];
    uint8_t E = data[2 + i * 2 + 1];
    blocks[i] = (2.56f * D) + (0.01f * E) - 327.68f;
  }

  Serial.printf("[BUS%d] Blocks:", bus);
  float sum = 0.0;
  for (int i = 0; i < 14; i++) {
    Serial.printf(" %.2f", blocks[i]);
    sum += blocks[i];
  }
  Serial.printf("\n[BUS%d] Sum of blocks: %.2f V\n", bus, sum);
}



void parseCAN(const CANMessage &frame, int bus) {
  float &packVoltage = (bus==1) ? packVoltage1 : packVoltage2;
  float &packCurrent = (bus==1) ? packCurrent1 : packCurrent2;
  int &soc = (bus==1) ? soc1 : soc2;
  int &ccl = (bus==1) ? ccl1 : ccl2;
  int &cdl = (bus==1) ? cdl1 : cdl2;
  int &tempA = (bus==1) ? temp1a : temp2a;
  int &tempB = (bus==1) ? temp1b : temp2b;
  uint16_t &faultCode = (bus==1) ? faultCode1 : faultCode2;
  uint8_t &deltaSOC = (bus==1) ? deltaSOC1 : deltaSOC2;
  uint16_t &calibX = (bus==1) ? calibX1 : calibX2;
  uint16_t &calibY = (bus==1) ? calibY1 : calibY2;
  uint16_t &calibZ = (bus==1) ? calibZ1 : calibZ2;
  uint8_t &flags = (bus==1) ? flags1 : flags2;

  switch (frame.id) {
    case 0x03B: { int16_t raw = ((frame.data[0]&0x0F)<<8)|frame.data[1];
      if (raw & 0x800) raw -= 0x1000; packCurrent = raw*0.1;
      packVoltage = (frame.data[2]<<8)|frame.data[3]; break; }
    case 0x3CB: { cdl=frame.data[0]; ccl=frame.data[1]; deltaSOC=frame.data[2];
      soc=frame.data[3]; tempA=(int8_t)frame.data[4]; tempB=(int8_t)frame.data[5]; break; }
    case 0x3CD: { faultCode=(frame.data[0]<<8)|frame.data[1];
      packVoltage=(frame.data[2]<<8)|frame.data[3]; break; }
    case 0x3C9: { calibY=(frame.data[0]<<4)|(frame.data[1]>>4);
      calibZ=((frame.data[1]&0x0F)<<8)|frame.data[2];
      calibX=(frame.data[3]<<4)|(frame.data[4]>>4); break; }
    case 0x4D1: { flags = frame.data[7]; break; }
  }

  if (frame.id==0x7EB) {
    uint8_t* buf = (bus==1) ? isoTPBuf1 : isoTPBuf2;
    size_t &isoLen = (bus==1) ? isoTPLen1 : isoTPLen2;
    uint8_t pci = frame.data[0];
    if ((pci & 0xF0)==0x00) {
      isoLen = pci&0x0F; memcpy(buf, frame.data+1, isoLen);
      if (buf[1]==0x61 && (buf[2]&0xF0)==0x40) parsePIDMap(buf, isoLen);
      else decodeBlocks(buf+3, isoLen-3, bus);
    } else if ((pci&0xF0)==0x10) {
      isoLen=frame.data[1]; size_t cpy=frame.len-2;
      memcpy(buf, frame.data+2, cpy); isoLen=cpy;
      
      // Send Flow Control
      CANMessage fc;
      fc.id = 0x7E3;
      fc.len = 8;
      fc.data[0] = 0x30; // Flow Control
      fc.data[1] = 0x00; // No block size
      fc.data[2] = 0x05; // 5 ms
      for (int i = 3; i < 8; i++) fc.data[i] = 0;
      if (bus == 1)
        ACAN_ESP32::can.tryToSend(fc);
      else
        can2.sendMsgBuf(fc.id, 0, fc.len, fc.data);
    } else if ((pci&0xF0)==0x20) {
      size_t cpy=frame.len-1; memcpy(buf+isoLen, frame.data+1, cpy);
      isoLen+=cpy; if (isoLen>=14) decodeBlocks(buf+3, isoLen-3, bus);
    }
  }
}
