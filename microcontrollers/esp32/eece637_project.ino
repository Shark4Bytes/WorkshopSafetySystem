#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// ---------------- Pin Setup ----------------
static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;

static const int LORA_RX_PIN = 18;   // ESP32 RX  <- LoRa TX
static const int LORA_TX_PIN = 17;   // ESP32 TX  -> LoRa RX

static const int LED_R_PIN = 4;
static const int LED_G_PIN = 5;
static const int LED_B_PIN = 6;

static const int BUZZER_PIN = 7;
static const int BUZZER_FREQ = 2000;
static const int BUZZER_RES = 8;

static const int SOUND_PIN = 1;   // DFRobot analog sound sensor

// ---------------- UART and Sensor ----------------
HardwareSerial LoRaSerial(2);
Adafruit_BME680 bme;
static const uint16_t SOUND_SAMPLE_WINDOW_MS = 50;


// ---------------- LoRa Config ----------------
static const uint32_t LORA_BAUD = 115200;
static const uint16_t LORA_DEST_ADDR = 2;

// ---------------- Timings ----------------
static uint32_t lastSendMs = 0;
static const uint32_t SEND_INTERVAL_MS = 3000;

static uint32_t lastAlertMs = 0;
static const uint32_t ALERT_INTERVAL_MS = 3000;

// ---------------- Alert State ----------------
static uint8_t activeErrorCount = 0;
static uint8_t activeErrorIndex = 0;
static char activeErrorCodes[8][8];
static bool alertLedOn = false;
static uint32_t alertPhaseStartMs = 0;

static const uint32_t ALERT_ON_MS = 100;
static const uint32_t ALERT_OFF_MS = 200;

// ---------------- LED Colors ----------------
enum LedColor
{
  LED_OFF,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  LED_YELLOW,
  LED_CYAN,
  LED_MAGENTA,
  LED_WHITE
};

// ---------------- LoRa Receive Packet ----------------
struct LoRaRcvPacket
{
  uint16_t srcAddr;
  uint16_t payloadLen;
  char payload[256];
  int rssi;
  int snr;
};

// ---------------- Prototypes ----------------
void loraSendStr(const char *s);
void loraSendCmd(const char *cmd);
bool loraInit(void);
bool loraSendText(uint16_t dest, const char *text);
bool loraReadLine(char *buf, size_t maxlen);
void handleLoraLine(const char *line);

bool bmeInit(void);
bool buildBmePacket(char *msg, size_t msgSize);

bool parseLoraRcvLine(const char *line, LoRaRcvPacket *pkt);

void setRgbLed(LedColor color);
void testRGBLed(void);

void buzzerTone(uint16_t freq);
void buzzerOff(void);
void beepBuzzer(uint16_t durationMs);

LedColor colorFromCode(const char *code);

bool parseErrorPayload(const char *payload, uint8_t *errCount, char codes[][8], uint8_t maxCodes);
void updateAlertState(const char *payload);
void processAlertBlink(void);
void handlePacketFromAddr2(const char *payload);

void soundInit(void);
uint16_t readSoundPeakToPeakMv(uint16_t sampleWindowMs);
uint8_t soundLevelToDb(uint16_t p2pMv);

// ---------------- Setup ----------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println("Starting ESP32-S3 + BME680 + LoRa");

  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  soundInit();
  LoRaSerial.setRxBufferSize(1024);
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  if (!bmeInit())
  {
    Serial.println("BME680 init failed");
    while (true)
    {
      delay(1000);
    }
  }

  while (!loraInit())
  {
    Serial.println("LoRa init failed, retrying...");
    delay(500);
  }

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  // Common anode OFF = HIGH
  digitalWrite(LED_R_PIN, HIGH);
  digitalWrite(LED_G_PIN, HIGH);
  digitalWrite(LED_B_PIN, HIGH);
  delay(10);
  setRgbLed(LED_OFF);

  testRGBLed();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, BUZZER_RES);

  Serial.println("Setup complete");
}

// ---------------- Main Loop ----------------
void loop()
{
  char line[256];

  while (loraReadLine(line, sizeof(line)))
  {
  handleLoraLine(line);
  }

  if (millis() - lastSendMs >= SEND_INTERVAL_MS)
  {
    char msg[128];
    lastSendMs = millis();

    if (buildBmePacket(msg, sizeof(msg)))
    {
      if (loraSendText(LORA_DEST_ADDR, msg))
      {
        Serial.print("Sent: ");
        Serial.println(msg);
      }
      else
      {
        Serial.println("LoRa send failed");
      }
    }
    else
    {
      Serial.println("Failed to read BME680");
    }
  }

  while (loraReadLine(line, sizeof(line)))
  {
  handleLoraLine(line);
  }

  processAlertBlink();
}

// ---------------- BME680 ----------------
bool bmeInit(void)
{
  if (!bme.begin(0x77, &Wire))
  {
    Serial.println("BME680 not found at 0x77");
    return false;
  }

  bme.setTemperatureOversampling(BME680_OS_2X);
  bme.setHumidityOversampling(BME680_OS_16X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(300, 100);

  Serial.println("BME680 init OK");
  return true;
}

bool buildBmePacket(char *msg, size_t msgSize)
{
  if (msg == nullptr || msgSize == 0)
    return false;

  if (!bme.performReading())
    return false;

  uint16_t soundP2pMv = readSoundPeakToPeakMv(SOUND_SAMPLE_WINDOW_MS);
  uint8_t db = soundLevelToDb(soundP2pMv);

  snprintf(
    msg,
    msgSize,
    "[TMP=%.2f;HUM=%.2f;PRS=%lu;DB=%u;GAS=%lu]",
    bme.temperature,
    bme.humidity,
    (unsigned long)bme.pressure,
    (unsigned int)db,
    (unsigned long)bme.gas_resistance
  );

  return true;
}

// ---------------- LoRa ----------------
void loraSendStr(const char *s)
{
  while (*s)
  {
    LoRaSerial.write(*s++);
  }
}

void loraSendCmd(const char *cmd)
{
  loraSendStr(cmd);
  loraSendStr("\r\n");
}

bool loraInit(void)
{
  loraSendCmd("AT");
  delay(100);

  loraSendCmd("AT+ADDRESS=1");
  delay(100);

  loraSendCmd("AT+NETWORKID=6");
  delay(100);

  loraSendCmd("AT+BAND=915000000");
  delay(100);

  loraSendCmd("AT+PARAMETER=9,7,1,12");
  delay(100);

  Serial.println("LoRa init commands sent");
  return true;
}

bool loraSendText(uint16_t dest, const char *text)
{
  char cmd[300];
  size_t len = strlen(text);

  if (len > 240)
    return false;

  snprintf(cmd, sizeof(cmd), "AT+SEND=%u,%u,%s", dest, (unsigned)len, text);
  loraSendCmd(cmd);
  return true;
}

bool loraReadLine(char *buf, size_t maxlen)
{
  static size_t idx = 0;

  while (LoRaSerial.available())
  {
    char c = (char)LoRaSerial.read();

    if (c == '\r')
      continue;

    if (c == '\n')
    {
      if (idx > 0)
      {
        buf[idx] = '\0';
        idx = 0;
        return true;
      }
    }
    else if (idx < (maxlen - 1))
    {
      buf[idx++] = c;
    }
  }

  return false;
}

bool parseLoraRcvLine(const char *line, LoRaRcvPacket *pkt)
{
  if (line == nullptr || pkt == nullptr)
    return false;

  if (strncmp(line, "+RCV=", 5) != 0)
    return false;

  const char *p = line + 5;
  char *endPtr;

  pkt->srcAddr = (uint16_t)strtoul(p, &endPtr, 10);
  if (*endPtr != ',')
    return false;

  p = endPtr + 1;
  pkt->payloadLen = (uint16_t)strtoul(p, &endPtr, 10);
  if (*endPtr != ',')
    return false;

  p = endPtr + 1;

  if (pkt->payloadLen >= sizeof(pkt->payload))
    return false;

  memcpy(pkt->payload, p, pkt->payloadLen);
  pkt->payload[pkt->payloadLen] = '\0';

  p += pkt->payloadLen;
  if (*p != ',')
    return false;

  p++;
  pkt->rssi = (int)strtol(p, &endPtr, 10);
  if (*endPtr != ',')
    return false;

  p = endPtr + 1;
  pkt->snr = (int)strtol(p, &endPtr, 10);

  return true;
}

void handleLoraLine(const char *line)
{
  Serial.print("LoRa: ");
  Serial.println(line);

  if (strcmp(line, "+OK") == 0)
  {
    Serial.println("Send confirmed");
  }
  else if (strncmp(line, "+RCV=", 5) == 0)
  {
    Serial.println("Received packet");
    LoRaRcvPacket pkt;

    if (parseLoraRcvLine(line, &pkt))
    {
      Serial.print("From addr ");
      Serial.print(pkt.srcAddr);
      Serial.print(" payload=");
      Serial.println(pkt.payload);

      if (pkt.srcAddr == 2)
      {
        handlePacketFromAddr2(pkt.payload);
      }
    }
    else
    {
      Serial.println("Failed to parse +RCV line");
    }
  }
}

// ---------------- LED ----------------
void setRgbLed(LedColor color)
{
  bool r = false;
  bool g = false;
  bool b = false;

  switch (color)
  {
    case LED_RED:     r = true; break;
    case LED_GREEN:   g = true; break;
    case LED_BLUE:    b = true; break;
    case LED_YELLOW:  r = true; g = true; break;
    case LED_CYAN:    g = true; b = true; break;
    case LED_MAGENTA: r = true; b = true; break;
    case LED_WHITE:   r = true; g = true; b = true; break;
    case LED_OFF:
    default:
      break;
  }

  // Common anode: LOW = ON, HIGH = OFF
  digitalWrite(LED_R_PIN, r ? LOW : HIGH);
  digitalWrite(LED_G_PIN, g ? LOW : HIGH);
  digitalWrite(LED_B_PIN, b ? LOW : HIGH);
}

void testRGBLed(void)
{
  Serial.println("Starting LED test...");

  setRgbLed(LED_RED); delay(1000);
  setRgbLed(LED_GREEN); delay(1000);
  setRgbLed(LED_BLUE); delay(1000);
  setRgbLed(LED_YELLOW); delay(1000);
  setRgbLed(LED_CYAN); delay(1000);
  setRgbLed(LED_MAGENTA); delay(1000);
  setRgbLed(LED_WHITE); delay(1000);
  setRgbLed(LED_OFF);

  Serial.println("LED test done");
}

LedColor colorFromCode(const char *code)
{
  if (code == nullptr)
    return LED_OFF;

  if (strcmp(code, "0x01") == 0) return LED_RED;      // hot
  if (strcmp(code, "0x02") == 0) return LED_BLUE;     // cold
  if (strcmp(code, "0x04") == 0) return LED_GREEN;    // low prs
  if (strcmp(code, "0x08") == 0) return LED_GREEN;    // high prs
  if (strcmp(code, "0x10") == 0) return LED_CYAN;     // humid
  if (strcmp(code, "0x20") == 0) return LED_MAGENTA;  // dry
  if (strcmp(code, "0x40") == 0) return LED_YELLOW;   // loud
  if (strcmp(code, "0x80") == 0) return LED_WHITE;    // gas

  return LED_OFF;
}

// ---------------- Buzzer ----------------
void buzzerTone(uint16_t freq)
{
  ledcWriteTone(BUZZER_PIN, freq);
}

void buzzerOff(void)
{
  ledcWriteTone(BUZZER_PIN, 0);
}



// ---------------- Error Payload Parsing ----------------
bool parseErrorPayload(const char *payload, uint8_t *errCount, char codes[][8], uint8_t maxCodes)
{
  if (payload == nullptr || errCount == nullptr || codes == nullptr)
    return false;

  // Accept both "err=" and "er="
  if (strncmp(payload, "err=", 4) == 0)
  {
    payload += 4;
  }
  else if (strncmp(payload, "errs=", 5) == 0)
  {
    payload += 5;
  }
  else
  {
    return false;
  }

  char *endPtr;
  unsigned long parsedErr = strtoul(payload, &endPtr, 10);

  // Real format uses ';'
  if (*endPtr != ';')
    return false;

  const char *p = endPtr + 1;

  if (strncmp(p, "codes=", 6) != 0)
    return false;

  p += 6;

  *errCount = 0;

  // Handle no-error case: err=0;codes=
  if (*p == '\0')
  {
    if (parsedErr != 0)
    {
      Serial.print("Warning: err count mismatch. err=");
      Serial.print(parsedErr);
      Serial.println(" parsed=0");
    }
    return true;
  }

  while (*p != '\0' && *errCount < maxCodes)
  {
    uint8_t i = 0;

    while (*p != ',' && *p != '\0' && i < 7)
    {
      codes[*errCount][i++] = *p++;
    }
    codes[*errCount][i] = '\0';

    if (i > 0)
      (*errCount)++;

    if (*p == ',')
      p++;
  }

  if (parsedErr != *errCount)
  {
    Serial.print("Warning: err count mismatch. err=");
    Serial.print(parsedErr);
    Serial.print(" parsed=");
    Serial.println(*errCount);
  }

  return true;
}

void updateAlertState(const char *payload)
{
  uint8_t parsedCount = 0;
  char parsedCodes[8][8];

  if (!parseErrorPayload(payload, &parsedCount, parsedCodes, 8))
  {
    Serial.println("Failed to parse error payload");
    return;
  }

  activeErrorCount = parsedCount;
  activeErrorIndex = 0;
  lastAlertMs = 0;

  for (uint8_t i = 0; i < activeErrorCount; i++)
  {
    strncpy(activeErrorCodes[i], parsedCodes[i], sizeof(activeErrorCodes[i]) - 1);
    activeErrorCodes[i][sizeof(activeErrorCodes[i]) - 1] = '\0';
  }

  if (activeErrorCount == 0)
  {
    setRgbLed(LED_OFF);
    buzzerOff();
    Serial.println("No active errors");
  }
  else
  {
    Serial.print("Stored ");
    Serial.print(activeErrorCount);
    Serial.println(" active error code(s)");
  }
}

void processAlertBlink(void)
{
  if (activeErrorCount == 0)
    return;

  uint32_t now = millis();

  if (!alertLedOn)
  {
    if (now - lastAlertMs < ALERT_INTERVAL_MS)
      return;

    LedColor color = colorFromCode(activeErrorCodes[activeErrorIndex]);

    setRgbLed(color);
    buzzerTone(2000);

    alertLedOn = true;
    alertPhaseStartMs = now;
  }
  else
  {
    if (now - alertPhaseStartMs < ALERT_ON_MS)
      return;

    setRgbLed(LED_OFF);
    buzzerOff();

    alertLedOn = false;
    lastAlertMs = now;

    activeErrorIndex++;
    if (activeErrorIndex >= activeErrorCount)
      activeErrorIndex = 0;
  }
}

void handlePacketFromAddr2(const char *payload)
{
  Serial.print("Addr2 payload: ");
  Serial.println(payload);

  updateAlertState(payload);
}

uint8_t soundLevelToDb(uint16_t p2pMv)
{
  if (p2pMv < 10)
    return 0;

  float rawDb = 20.0f * log10((float)p2pMv);

  // Calibrated from previous DB=56 reads closer to 70
  float dbMin = -22.5f;
  float dbMax = 85.0f;

  float db = ((rawDb - dbMin) * 100.0f) / (dbMax - dbMin);

  if (db < 0.0f)
    db = 0.0f;

  if (db > 100.0f)
    db = 100.0f;

  return (uint8_t)(db + 0.5f);
}

uint16_t readSoundPeakToPeakMv(uint16_t sampleWindowMs)
{
  uint32_t startMs = millis();
  uint32_t signalMax = 0;
  uint32_t signalMin = 0xFFFFFFFF;

  while (millis() - startMs < sampleWindowMs)
  {
    uint32_t mv = analogReadMilliVolts(SOUND_PIN);

    if (mv > signalMax)
      signalMax = mv;

    if (mv < signalMin)
      signalMin = mv;

    delayMicroseconds(200);
  }

  if (signalMax < signalMin)
    return 0;

  return (uint16_t)(signalMax - signalMin);
}

void soundInit(void)
{
  pinMode(SOUND_PIN, INPUT);

  // ESP32-S3 ADC is 12-bit in normal single-read mode.
  analogReadResolution(12);

  // Highest Arduino-ESP32 attenuation range.
  // Still keep the sensor powered from 3.3V.
  analogSetPinAttenuation(SOUND_PIN, ADC_11db);

  // Throw away a few startup reads.
  for (uint8_t i = 0; i < 5; i++)
  {
    analogRead(SOUND_PIN);
    delay(2);
  }

  Serial.println("Sound sensor init OK");
}
