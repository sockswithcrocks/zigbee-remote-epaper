#ifndef ZIGBEE_MODE_ED
#error "Select Zigbee ED (end device) in Tools -> Zigbee mode"
#endif

#include <Zigbee.h>
#include <SPI.h>
#include <Wire.h>
#include <GxEPD2_BW.h>
#include <gdey/GxEPD2_270_GDEY027T91.h>
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/Org_01.h>

#define EPD_CS    18
#define EPD_DC     4
#define EPD_RST    5
#define EPD_BUSY   6
#define EPD_SCK   21
#define EPD_MOSI  19
#define EPD_MISO  -1

#define DISPLAY_ROTATION 0
#define BUTTON_COUNT 10
#define LONG_PRESS_MS 1200
#define BUTTON_DEBOUNCE_MS 2
#define FACTORY_RESET_HOLD_MS 5000
#define PAGE_MIN 1
#define PAGE_MAX 4
#define PAGE_SLOT_COUNT 11
#define PAGE_SLOT_LEN 24
#define DISPLAY_REFRESH_IDLE_MS 180
#define ENABLE_ZIGBEE_START 0

#define EVENT_SHORT 1
#define EVENT_LONG  2

#define ZIGBEE_LEVEL_MOVE_UP   0
#define ZIGBEE_LEVEL_MOVE_DOWN 1
#define ZIGBEE_LEVEL_STEP_UP   0
#define ZIGBEE_LEVEL_STEP_DOWN 1

#define I2C_SDA_PIN 17
#define I2C_SCL_PIN 16
#define MCP23017_ADDR_DEFAULT 0x27
#define MCP23017_IODIRB 0x01
#define MCP23017_GPPUB  0x0D
#define MCP23017_GPIOB  0x13
#define MCP23017_BUTTON10_BIT 7

#define IQS550_ADDR                 0x74
#define IQS550_INT_PIN              15
#define IQS550_RST_PIN              9

static const int8_t BUTTON_PINS[BUTTON_COUNT] = {7, 1, 23, 22, 10, 11, 2, 20, 3, -1};
static const char *PAGE_NAMES[PAGE_MAX] = {"Klima", "Licht", "AppleTV", "Receiver"};

GxEPD2_BW<GxEPD2_270_GDEY027T91, GxEPD2_270_GDEY027T91::HEIGHT> display(
  GxEPD2_270_GDEY027T91(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY)
);

ZigbeeColorDimmerSwitch g_remoteEndpoint(1);
uint8_t g_mcp23017Addr = MCP23017_ADDR_DEFAULT;
bool g_mcp23017Found = false;
bool g_iqs550Found = false;
volatile bool g_iqs550Rdy = false;
String g_iqsStatus = "IQS boot";
String g_iqsProbeStatus = "IQS idle";

void IRAM_ATTR iqs550ISR() {
  g_iqs550Rdy = true;
}

static void requestDisplayRefresh(unsigned long delayMs = DISPLAY_REFRESH_IDLE_MS);

static bool mcpWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(g_mcp23017Addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool mcpReadRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(g_mcp23017Addr);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }
  if (Wire.requestFrom(g_mcp23017Addr, (uint8_t)1) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

static bool detectMcp23017() {
  for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      g_mcp23017Addr = addr;
      Serial.printf("MCP23017 found at 0x%02X\n", g_mcp23017Addr);
      return true;
    }
  }
  Serial.println("MCP23017 not found on 0x20..0x27");
  return false;
}

static void initMcp23017() {
  g_mcp23017Found = detectMcp23017();
  if (!g_mcp23017Found) {
    return;
  }

  uint8_t iodirb = 0xFF;
  uint8_t gppub = (1 << MCP23017_BUTTON10_BIT);

  mcpWriteRegister(MCP23017_IODIRB, iodirb);
  mcpWriteRegister(MCP23017_GPPUB, gppub);
}

// ZCL-Command-Helfer
//
// Wichtig: address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT
//
// Mit einer hardcodierten Zieladresse (0x0000/EP1) wuerde ZHA den Command
// als eingehenden Steuerbefehl fuer ein anderes Geraet interpretieren und
// nicht als Button-Event dieses Remotes loggen.
//
// Ohne Zieladresse (NOT_PRESENT) sendet der ESP den Command ins Netz und
// ZHA ordnet ihn dem absendenden Geraet zu. Das ist die Voraussetzung,
// damit device_automation_triggers im Quirk greifen koennen.
static inline esp_zb_zcl_basic_cmd_t makeBasicCmd() {
  esp_zb_zcl_basic_cmd_t cmd = {};
  cmd.src_endpoint = g_remoteEndpoint.getEndpoint();
  return cmd;
}

static void sendOnOffCommand(uint8_t commandId) {
  esp_zb_zcl_on_off_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.on_off_cmd_id = commandId;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_on_off_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendIdentifyCommand(uint16_t identifyTime) {
  esp_zb_zcl_identify_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.identify_time = identifyTime;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_identify_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveToLevel(uint8_t level) {
  esp_zb_zcl_move_to_level_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.level = level;
  cmd_req.transition_time = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_move_to_level_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendLevelStep(uint8_t stepMode, uint8_t stepSize, uint16_t transitionTime) {
  esp_zb_zcl_level_step_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.step_mode = stepMode;
  cmd_req.step_size = stepSize;
  cmd_req.transition_time = transitionTime;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_step_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveToColor(uint16_t x, uint16_t y) {
  esp_zb_zcl_color_move_to_color_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.color_x = x;
  cmd_req.color_y = y;
  cmd_req.transition_time = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_to_color_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendOffWithEffect() {
  esp_zb_zcl_on_off_off_with_effect_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.effect_id = 0;
  cmd_req.effect_variant = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_on_off_off_with_effect_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveToLevelWithOnOff(uint8_t level) {
  esp_zb_zcl_move_to_level_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.level = level;
  cmd_req.transition_time = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_move_to_level_with_onoff_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendLevelMove(uint8_t moveMode) {
  esp_zb_zcl_level_move_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.move_mode = moveMode;
  cmd_req.rate = 50;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_move_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendLevelStop() {
  esp_zb_zcl_level_stop_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_stop_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendLevelStepWithOnOff(uint8_t stepMode, uint8_t stepSize) {
  esp_zb_zcl_level_step_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.step_mode = stepMode;
  cmd_req.step_size = stepSize;
  cmd_req.transition_time = 1;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_step_with_onoff_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendLevelMoveWithOnOff(uint8_t moveMode) {
  esp_zb_zcl_level_move_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.move_mode = moveMode;
  cmd_req.rate = 50;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_level_move_with_onoff_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveColor(int16_t rateX, int16_t rateY) {
  esp_zb_zcl_color_move_color_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.rate_x = rateX;
  cmd_req.rate_y = rateY;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_color_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveToColorTemp(uint16_t mireds) {
  esp_zb_zcl_color_move_to_color_temperature_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.color_temperature = mireds;
  cmd_req.transition_time = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_to_color_temperature_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveColorTemp(uint8_t moveMode, uint16_t rate) {
  esp_zb_zcl_color_move_color_temperature_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.move_mode = moveMode;
  cmd_req.rate = rate;
  cmd_req.color_temperature_minimum = 153;
  cmd_req.color_temperature_maximum = 500;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_color_temperature_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveToHue(uint8_t hue) {
  esp_zb_zcl_color_move_to_hue_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.hue = hue;
  cmd_req.direction = 0;
  cmd_req.transition_time = 0;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_to_hue_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

static void sendMoveHue(uint8_t moveMode) {
  esp_zb_zcl_color_move_hue_cmd_t cmd_req = {};
  cmd_req.zcl_basic_cmd = makeBasicCmd();
  cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  cmd_req.move_mode = moveMode;
  cmd_req.rate = 50;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_color_move_hue_cmd_req(&cmd_req);
  esp_zb_lock_release();
}

bool g_buttonPressed[BUTTON_COUNT] = {false};
bool g_buttonLastReading[BUTTON_COUNT] = {false};
unsigned long g_buttonLastDebounceMs[BUTTON_COUNT] = {0};
unsigned long g_buttonPressedSinceMs[BUTTON_COUNT] = {0};
bool g_buttonLongSent[BUTTON_COUNT] = {false};
uint32_t g_buttonPressCount[BUTTON_COUNT] = {0};

uint8_t g_currentPage = 1;
String g_lastAction = "Boot";
bool g_zigbeeReady = false;
unsigned long g_lastDisplayRefreshMs = 0;
unsigned long g_factoryResetSinceMs = 0;
bool g_factoryResetTriggered = false;
bool g_displayBorder = true;
bool g_displayRefreshPending = false;
unsigned long g_displayRefreshDueMs = 0;
unsigned long g_lastZigbeeRetryMs = 0;
char g_pageSlots[PAGE_MAX][PAGE_SLOT_COUNT][PAGE_SLOT_LEN] = {{{0}}};
uint8_t g_lastMcpGpioB = 0xFF;
unsigned long g_lastMcpPollMs = 0;

static void reportCurrentPageState() {
  if (!(g_zigbeeReady && Zigbee.connected())) {
    return;
  }
  sendMoveToLevel(200 + g_currentPage);
}

static void setPageSlot(uint8_t pageIndex, uint8_t slotIndex, const char *value) {
  if (pageIndex >= PAGE_MAX || slotIndex >= PAGE_SLOT_COUNT) {
    return;
  }

  snprintf(g_pageSlots[pageIndex][slotIndex], PAGE_SLOT_LEN, "%s", value ? value : "");
}

static void initDisplayState() {
  for (uint8_t page = 0; page < PAGE_MAX; page++) {
    for (uint8_t slot = 0; slot < PAGE_SLOT_COUNT; slot++) {
      g_pageSlots[page][slot][0] = '\0';
    }
  }

  setPageSlot(0, 0, "Klima");
  setPageSlot(0, 1, "Wetter");
  setPageSlot(0, 2, "--.-C");
  setPageSlot(0, 3, "Heizung");
  setPageSlot(0, 4, "Soll");
  setPageSlot(0, 5, "Fenster");
  setPageSlot(0, 6, "icon:hot");
  setPageSlot(0, 7, "offline");
  setPageSlot(0, 8, "icon:cold");
  setPageSlot(0, 9, "icon:mode-");
  setPageSlot(0, 10, "icon:mode+");

  setPageSlot(1, 0, "Licht");
  setPageSlot(1, 1, "Szene");
  setPageSlot(1, 2, "off");
  setPageSlot(1, 3, "Helligkeit");
  setPageSlot(1, 4, "--");
  setPageSlot(1, 5, "Info");
  setPageSlot(1, 6, "icon:up");
  setPageSlot(1, 7, "offline");
  setPageSlot(1, 8, "icon:down");
  setPageSlot(1, 9, "icon:left");
  setPageSlot(1, 10, "icon:right");

  setPageSlot(2, 0, "AppleTV");
  setPageSlot(2, 1, "Quelle");
  setPageSlot(2, 2, "none");
  setPageSlot(2, 3, "Back");
  setPageSlot(2, 4, "Home");
  setPageSlot(2, 5, "Select");
  setPageSlot(2, 6, "Up");
  setPageSlot(2, 7, "offline");
  setPageSlot(2, 8, "Down");
  setPageSlot(2, 9, "Left");
  setPageSlot(2, 10, "Right");

  setPageSlot(3, 0, "Receiver");
  setPageSlot(3, 1, "Quelle");
  setPageSlot(3, 2, "none");
  setPageSlot(3, 3, "Back");
  setPageSlot(3, 4, "Home");
  setPageSlot(3, 5, "Mute");
  setPageSlot(3, 6, "Vol+");
  setPageSlot(3, 7, "offline");
  setPageSlot(3, 8, "Vol-");
  setPageSlot(3, 9, "Input-");
  setPageSlot(3, 10, "Input+");
}

static void syncDebugDisplayState() {
  setPageSlot(g_currentPage - 1, 0, "IQS DBG");
  setPageSlot(g_currentPage - 1, 1, g_iqsStatus.c_str());
  setPageSlot(g_currentPage - 1, 2, g_iqsProbeStatus.c_str());
  setPageSlot(g_currentPage - 1, 3, g_lastAction.c_str());
  char mcpInfo[PAGE_SLOT_LEN];
  if (g_mcp23017Found) {
    snprintf(mcpInfo, sizeof(mcpInfo), "MCP %02X B:%02X", g_mcp23017Addr, g_lastMcpGpioB);
  } else {
    snprintf(mcpInfo, sizeof(mcpInfo), "MCP fail");
  }
  setPageSlot(g_currentPage - 1, 4, mcpInfo);
  setPageSlot(g_currentPage - 1, 5, "touch now");
  setPageSlot(g_currentPage - 1, 6, "watch IQS");
  setPageSlot(g_currentPage - 1, 7, "slot7");
  setPageSlot(g_currentPage - 1, 8, "slot8");
}

static void iqs550ResetPulse() {
  pinMode(IQS550_RST_PIN, OUTPUT);
  digitalWrite(IQS550_RST_PIN, LOW);
  delay(1000);
  digitalWrite(IQS550_RST_PIN, HIGH);
  delay(1000);
}

static bool iqs550WaitReadyWindow(uint16_t timeoutMs = 1500) {
  unsigned long startMs = millis();
  while (digitalRead(IQS550_INT_PIN) == HIGH) {
    if (millis() - startMs > timeoutMs) {
      return false;
    }
    delay(1);
  }
  while (digitalRead(IQS550_INT_PIN) == LOW) {
    if (millis() - startMs > timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

static bool iqs550WriteRegister(uint8_t reg, const uint8_t *data, uint8_t len) {
  uint8_t lastErr = 0;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    if (!iqs550WaitReadyWindow()) {
      lastErr = 9;
      delay(20);
      continue;
    }
    Wire.beginTransmission(IQS550_ADDR);
    Wire.write(reg);
    for (uint8_t i = 0; i < len; i++) {
      Wire.write(data[i]);
    }
    lastErr = Wire.endTransmission(true);
    if (lastErr == 0) {
      return true;
    }
    delay(20);
  }
  g_iqsStatus = (lastErr == 9) ? "IQS nRDY" : ("IQS we" + String(lastErr));
  return false;
}

static bool iqs550ReadRegister(uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t lastErr = 0;
  uint8_t lastRcnt = 0;
  bool waitTimedOut = false;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    if (!iqs550WaitReadyWindow()) {
      lastErr = 9;
      waitTimedOut = true;
      delay(20);
      continue;
    }
    Wire.beginTransmission(IQS550_ADDR);
    Wire.write(reg);
    lastErr = Wire.endTransmission(false);
    if (lastErr != 0) {
      delay(20);
      continue;
    }
    lastRcnt = Wire.requestFrom((uint8_t)IQS550_ADDR, len, (uint8_t)true);
    if (lastRcnt == len) {
      for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
      }
      return true;
    }
    while (Wire.available()) Wire.read();
    delay(20);
  }
  if (waitTimedOut) {
    g_iqsStatus = "IQS nRDY";
    g_iqsProbeStatus = "wait win";
  } else if (lastErr != 0) {
    g_iqsStatus = "IQS re" + String(lastErr);
    g_iqsProbeStatus = "prep rd";
  } else if (lastRcnt != 0) {
    g_iqsStatus = "IQS rc" + String(lastRcnt);
    g_iqsProbeStatus = "short rd";
  } else if (lastErr != 0) {
    g_iqsStatus = "IQS r?";
    g_iqsProbeStatus = "unk fail";
  } else {
    g_iqsStatus = "IQS rc0";
    g_iqsProbeStatus = "no bytes";
  }
  return false;
}

static bool iqs550ReadRegisterNoWait(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(IQS550_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    g_iqsStatus = "NW re" + String(err);
    return false;
  }

  uint8_t rcnt = Wire.requestFrom((uint8_t)IQS550_ADDR, len, (uint8_t)true);
  if (rcnt != len) {
    while (Wire.available()) Wire.read();
    g_iqsStatus = "NW rc" + String(rcnt);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

static void drawSlotText(int16_t x, int16_t y, const char *text) {
  display.setCursor(x, y);
  display.print(text ? text : "");
}

static bool readButtonRaw(uint8_t idx) {
  if (idx == 9) {
    if (!g_mcp23017Found) {
      return false;
    }
    return (g_lastMcpGpioB & (1 << MCP23017_BUTTON10_BIT)) == 0;
  }
  return digitalRead(BUTTON_PINS[idx]) == LOW;
}

static void pollMcpState() {
  if (!g_mcp23017Found) {
    return;
  }

  if (millis() - g_lastMcpPollMs < 50) {
    return;
  }
  g_lastMcpPollMs = millis();

  uint8_t gpiob = 0xFF;
  if (!mcpReadRegister(MCP23017_GPIOB, gpiob)) {
    Serial.println("MCP23017 read failed, reinit");
    Wire.end();
    delay(10);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    g_mcp23017Found = false;
    initMcp23017();
    return;
  }

  if (gpiob != g_lastMcpGpioB) {
    Serial.printf("MCP GPIOB changed: 0x%02X -> 0x%02X\n", g_lastMcpGpioB, gpiob);
    g_lastMcpGpioB = gpiob;
    g_lastAction = "MCP B:" + String(gpiob, HEX);
    requestDisplayRefresh(10);
  } else {
    g_lastMcpGpioB = gpiob;
  }
}

static bool anyButtonCurrentlyPressed() {
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if (g_buttonPressed[i]) {
      return true;
    }
  }
  return false;
}

static void requestDisplayRefresh(unsigned long delayMs) {
  g_displayRefreshPending = true;
  g_displayRefreshDueMs = millis() + delayMs;
}

static void refreshDisplay() {
  syncDebugDisplayState();
  char *page = g_pageSlots[g_currentPage - 1][0];

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    if (g_displayBorder) {
      display.drawRect(1, 1, 174, 40, GxEPD_BLACK);
      display.drawRect(1, 45, 85, 40, GxEPD_BLACK);
      display.drawRect(90, 45, 85, 40, GxEPD_BLACK);
      display.drawRect(1, 89, 85, 40, GxEPD_BLACK);
      display.drawRect(90, 89, 85, 40, GxEPD_BLACK);
      display.drawRect(1, 133, 85, 40, GxEPD_BLACK);
      display.drawRect(90, 133, 85, 40, GxEPD_BLACK);
      display.drawRect(1, 177, 85, 40, GxEPD_BLACK);
      display.drawRect(90, 177, 85, 40, GxEPD_BLACK);
      display.drawRect(1, 221, 85, 40, GxEPD_BLACK);
      display.drawRect(90, 221, 85, 40, GxEPD_BLACK);
    }

    display.setFont(&FreeMono12pt7b);
    display.setCursor(10, 27);
    display.print((page && page[0]) ? page : PAGE_NAMES[g_currentPage - 1]);

    display.setFont(&Org_01);
    drawSlotText(11, 60,  g_pageSlots[g_currentPage - 1][1]);
    drawSlotText(100, 60, g_pageSlots[g_currentPage - 1][2]);
    drawSlotText(11, 105, g_pageSlots[g_currentPage - 1][3]);
    drawSlotText(100, 105, g_pageSlots[g_currentPage - 1][4]);
    drawSlotText(11, 150, g_pageSlots[g_currentPage - 1][5]);
    drawSlotText(100, 150, g_pageSlots[g_currentPage - 1][6]);
    drawSlotText(11, 194, g_pageSlots[g_currentPage - 1][7]);
    drawSlotText(100, 194, g_pageSlots[g_currentPage - 1][8]);
    drawSlotText(11, 238, g_pageSlots[g_currentPage - 1][9]);
    drawSlotText(100, 238, g_pageSlots[g_currentPage - 1][10]);
  } while (display.nextPage());

  g_lastDisplayRefreshMs = millis();
}

static void sendZigbeeButtonCommand(uint8_t buttonIndex, uint8_t eventKind) {
  switch (buttonIndex + 1) {
    case 1:
      (eventKind == EVENT_SHORT)
        ? sendOnOffCommand(ESP_ZB_ZCL_CMD_ON_OFF_ON_ID)
        : sendOnOffCommand(ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID);
      break;
    case 2:
      (eventKind == EVENT_SHORT)
        ? sendOnOffCommand(ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID)
        : sendOffWithEffect();
      break;
    case 3:
      (eventKind == EVENT_SHORT)
        ? sendMoveToLevel(32)
        : sendMoveToLevel(48);
      break;
    case 4:
      (eventKind == EVENT_SHORT)
        ? sendMoveToLevel(64)
        : sendMoveToLevel(80);
      break;
    case 5:
      (eventKind == EVENT_SHORT)
        ? sendLevelStep(ZIGBEE_LEVEL_STEP_UP, 16, 1)
        : sendLevelStep(ZIGBEE_LEVEL_STEP_DOWN, 16, 1);
      break;
    case 6:
      (eventKind == EVENT_SHORT)
        ? sendLevelStep(ZIGBEE_LEVEL_STEP_UP, 32, 2)
        : sendLevelStep(ZIGBEE_LEVEL_STEP_DOWN, 32, 2);
      break;
    case 7:
      (eventKind == EVENT_SHORT)
        ? sendMoveToColor(0x1111, 0x1111)
        : sendMoveToColor(0x2222, 0x2222);
      break;
    case 8:
      (eventKind == EVENT_SHORT)
        ? sendMoveToColor(0x3333, 0x3333)
        : sendMoveToColor(0x4444, 0x4444);
      break;
    case 9:
      (eventKind == EVENT_SHORT)
        ? sendMoveToColor(0x5555, 0x5555)
        : sendMoveToColor(0x6666, 0x6666);
      break;
    case 10:
      break;
    default:
      break;
  }
}

static void handleButtonEvent(uint8_t buttonIndex, uint8_t eventKind) {
  g_lastAction = "B" + String(buttonIndex + 1)
               + (eventKind == EVENT_LONG ? " long" : " short");
  syncDebugDisplayState();

  if (buttonIndex == 0) {
    if (eventKind == EVENT_SHORT) {
      g_currentPage = (g_currentPage <= PAGE_MIN) ? PAGE_MAX : (g_currentPage - 1);
    } else {
      g_currentPage = 1;
    }
    reportCurrentPageState();
    requestDisplayRefresh();
    return;
  } else if (buttonIndex == 1) {
    if (eventKind == EVENT_SHORT) {
      g_currentPage = (g_currentPage >= PAGE_MAX) ? PAGE_MIN : (g_currentPage + 1);
    } else {
      g_currentPage = PAGE_MAX;
    }
    reportCurrentPageState();
    requestDisplayRefresh();
    return;
  }

  if (g_zigbeeReady && Zigbee.connected()) {
    sendZigbeeButtonCommand(buttonIndex, eventKind);
  }

  requestDisplayRefresh();
}

static void updateButtons() {
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    bool reading = readButtonRaw(i);
    if (reading != g_buttonLastReading[i]) {
      g_buttonLastDebounceMs[i] = millis();
      g_buttonLastReading[i] = reading;
    }

    if ((millis() - g_buttonLastDebounceMs[i]) < BUTTON_DEBOUNCE_MS) {
      continue;
    }

    if (reading != g_buttonPressed[i]) {
      g_buttonPressed[i] = reading;
      if (g_buttonPressed[i]) {
        g_buttonPressCount[i]++;
        g_buttonPressedSinceMs[i] = millis();
        g_buttonLongSent[i] = false;
      } else if (!g_buttonLongSent[i]) {
        handleButtonEvent(i, EVENT_SHORT);
      }
    }

    if (g_buttonPressed[i] && !g_buttonLongSent[i] &&
        (millis() - g_buttonPressedSinceMs[i] >= LONG_PRESS_MS)) {
      g_buttonLongSent[i] = true;
      handleButtonEvent(i, EVENT_LONG);
    }
  }
}

static void updateFactoryResetCombo() {
  bool comboPressed = readButtonRaw(0) && readButtonRaw(1);
  if (!comboPressed) {
    g_factoryResetSinceMs = 0;
    g_factoryResetTriggered = false;
    return;
  }

  if (g_factoryResetSinceMs == 0) {
    g_factoryResetSinceMs = millis();
    g_lastAction = "Reset armed";
    refreshDisplay();
    return;
  }

  if (!g_factoryResetTriggered &&
      (millis() - g_factoryResetSinceMs >= FACTORY_RESET_HOLD_MS)) {
    g_factoryResetTriggered = true;
    g_lastAction = "Factory reset";
    refreshDisplay();
    delay(300);
    Zigbee.factoryReset();
  }
}

static bool iqs550WaitReady(uint16_t timeoutMs = 300) {
  unsigned long t = millis();
  while (digitalRead(IQS550_INT_PIN) != LOW) {
    if (millis() - t > timeoutMs) return false;
    delay(1);
  }
  return true;
}

static void iqs550ReadIdleDiagnostics(uint8_t xyInfo, uint8_t idTag) {
  uint8_t prox[2] = {0};
  uint8_t touch[2] = {0};
  uint8_t snap[2] = {0};
  uint8_t bl = 0;

  bool proxOk = iqs550ReadRegister(0x02, prox, sizeof(prox));
  bool touchOk = iqs550ReadRegister(0x03, touch, sizeof(touch));
  bool snapOk = iqs550ReadRegister(0x08, snap, sizeof(snap));
  bool blOk = iqs550ReadRegister(0xFF, &bl, 1);

  char line1[24];
  snprintf(line1, sizeof(line1), "xy=%02X id=%02X", xyInfo, idTag);
  g_iqsStatus = line1;

  char line2[24];
  snprintf(
    line2,
    sizeof(line2),
    "p=%s%02X t=%s%02X",
    proxOk ? "" : "!",
    proxOk ? prox[0] : 0,
    touchOk ? "" : "!",
    touchOk ? touch[0] : 0
  );
  g_iqsProbeStatus = line2;

  char line3[24];
  snprintf(
    line3,
    sizeof(line3),
    "s=%s%02X bl=%s%02X",
    snapOk ? "" : "!",
    snapOk ? snap[0] : 0,
    blOk ? "" : "!",
    blOk ? bl : 0
  );
  g_lastAction = line3;

  Serial.printf(
    "IQS550 idle diag: xy=0x%02X id=0x%02X prox=%s0x%02X%02X touch=%s0x%02X%02X snap=%s0x%02X%02X bl=%s0x%02X\n",
    xyInfo,
    idTag,
    proxOk ? "" : "ERR ",
    prox[0],
    prox[1],
    touchOk ? "" : "ERR ",
    touch[0],
    touch[1],
    snapOk ? "" : "ERR ",
    snap[0],
    snap[1],
    blOk ? "" : "ERR ",
    bl
  );
}

static void initIqs550() {
  detachInterrupt(digitalPinToInterrupt(IQS550_INT_PIN));
  g_iqs550Found = false;
  g_iqs550Rdy = false;
  pinMode(IQS550_INT_PIN, INPUT_PULLUP);
  Wire.setTimeOut(500);

  g_iqsStatus = "IQS rst";
  g_iqsProbeStatus = "hold reset";
  iqs550ResetPulse();

  g_iqsStatus = (digitalRead(IQS550_INT_PIN) == LOW) ? "IQS ILO" : "IQS IHI";
  Serial.println(g_iqsStatus);

  if (!iqs550WaitReady(2000)) {
    g_iqsStatus = "IQS nRDY";
    g_iqsProbeStatus = "wait rdy";
    g_lastAction = g_iqsStatus;
    attachInterrupt(digitalPinToInterrupt(IQS550_INT_PIN), iqs550ISR, FALLING);
    g_iqs550Rdy = (digitalRead(IQS550_INT_PIN) == LOW);
    return;
  }

  uint8_t ch_setup[7] = {10, 13, 10, 13, 0x10, 0x1F, 0xFF};
  uint8_t ack_reset = 0x80;
  uint8_t threshold[6] = {10, 25, 5, 10, 0, 0};
  uint8_t control[2] = {0x20, 0x01};

  g_iqsStatus = "IQS ch";
  g_iqsProbeStatus = "wr 15";
  if (!iqs550WriteRegister(0x15, ch_setup, sizeof(ch_setup))) { g_lastAction = g_iqsStatus; return; }
  delay(1000);

  g_iqsStatus = "IQS ack";
  g_iqsProbeStatus = "wr 10";
  if (!iqs550WriteRegister(0x10, &ack_reset, 1)) { g_lastAction = g_iqsStatus; return; }
  delay(1000);

  g_iqsStatus = "IQS thr";
  g_iqsProbeStatus = "wr 11";
  if (!iqs550WriteRegister(0x11, threshold, sizeof(threshold))) { g_lastAction = g_iqsStatus; return; }
  delay(1000);

  g_iqsStatus = "IQS ctl";
  g_iqsProbeStatus = "wr 10";
  if (!iqs550WriteRegister(0x10, control, sizeof(control))) { g_lastAction = g_iqsStatus; return; }
  delay(1000);

  g_iqsStatus = "IQS ver";
  g_iqsProbeStatus = "rd 00";
  uint8_t ver[10] = {0};
  if (!iqs550ReadRegister(0x00, ver, sizeof(ver))) { g_lastAction = g_iqsStatus; return; }

  g_iqs550Found = true;
  g_iqsStatus = "IQS ok";
  g_iqsProbeStatus = "v=" + String(ver[4]) + "." + String(ver[5]);
  g_lastAction = "IQS ini";
  Serial.println("IQS550 init ok");
  attachInterrupt(digitalPinToInterrupt(IQS550_INT_PIN), iqs550ISR, FALLING);
  g_iqs550Rdy = (digitalRead(IQS550_INT_PIN) == LOW);
}

static void pollIqs550() {
  bool intLow = (digitalRead(IQS550_INT_PIN) == LOW);
  static unsigned long lastDebugPollMs = 0;
  static uint32_t pollCount = 0;

  if (!g_iqs550Found) {
    static unsigned long lastRetryMs = 0;
    if (millis() - lastRetryMs > 3000) {
      lastRetryMs = millis();
      g_iqsStatus = intLow ? "IQS ILO" : "IQS IHI";
      g_iqsProbeStatus = "retry init";
      initIqs550();
      requestDisplayRefresh(10);
    }
    return;
  }

  if (intLow) {
    g_iqs550Rdy = true;
  }
  if (!g_iqs550Rdy) {
    if (millis() - lastDebugPollMs < 250) {
      return;
    }
    lastDebugPollMs = millis();
    pollCount++;
    g_iqsStatus = intLow ? "DBG INT LO" : "DBG INT HI";
    g_iqsProbeStatus = "poll#" + String(pollCount);
    g_lastAction = "pre rd01";
  }
  g_iqs550Rdy = false;

  uint8_t xy[8] = {0};
  if (!iqs550ReadRegister(0x01, xy, sizeof(xy))) {
    g_lastAction = "rd01 fail";
    if (!iqs550ReadRegisterNoWait(0x01, xy, sizeof(xy))) {
      g_lastAction = "rd01 both";
      requestDisplayRefresh(10);
      return;
    }
    g_lastAction = "rd01 nwok";
  }

  {
    char rawLine[24];
    snprintf(rawLine, sizeof(rawLine), "%02X %02X %02X %02X", xy[0], xy[1], xy[2], xy[3]);
    g_iqsStatus = rawLine;
    snprintf(rawLine, sizeof(rawLine), "%02X %02X %02X %02X", xy[4], xy[5], xy[6], xy[7]);
    g_iqsProbeStatus = rawLine;
    g_lastAction = "rd01 ok";
  }

  if (xy[0] == 0x08 && xy[1] == 0x00 && xy[2] == 0xFF && xy[3] == 0xFF &&
      xy[4] == 0xFF && xy[5] == 0xFF && xy[6] == 0x00 && xy[7] == 0x00) {
    iqs550ReadIdleDiagnostics(xy[0], xy[1]);
    requestDisplayRefresh(10);
    return;
  }
  requestDisplayRefresh(10);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  initDisplayState();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  initMcp23017();
  initIqs550();

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if (BUTTON_PINS[i] >= 0) {
      pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }
    g_buttonPressed[i] = readButtonRaw(i);
    g_buttonLastReading[i] = g_buttonPressed[i];
  }

  SPI.begin(EPD_SCK, EPD_MISO, EPD_MOSI, EPD_CS);
  display.init(115200);
  display.setRotation(DISPLAY_ROTATION);
  refreshDisplay();

  #if ENABLE_ZIGBEE_START
    g_remoteEndpoint.setManufacturerAndModel("Ruben", "EpaperRemote");
    g_remoteEndpoint.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 37);
    Zigbee.addEndpoint(&g_remoteEndpoint);

    if (!Zigbee.begin()) {
      Serial.println("Zigbee failed to start");
      g_lastAction = "Zigbee start fail";
      refreshDisplay();
    } else {
      Serial.println("Joining Zigbee network...");
      unsigned long waitStartMs = millis();
      while (!Zigbee.connected() && (millis() - waitStartMs < 5000)) {
        updateFactoryResetCombo();
        updateButtons();
        if (g_displayRefreshPending &&
            millis() >= g_displayRefreshDueMs &&
            !anyButtonCurrentlyPressed()) {
          g_displayRefreshPending = false;
          refreshDisplay();
        }
        delay(10);
      }
    }

    g_zigbeeReady = Zigbee.connected();
    if (g_zigbeeReady) {
      reportCurrentPageState();
      g_lastAction = "Zigbee connected";
    } else {
      g_lastAction = "No Zigbee";
    }
  #else
    g_zigbeeReady = false;
    if (g_lastAction == "Boot") {
      g_lastAction = "Zigbee disabled";
    }
  #endif
  syncDebugDisplayState();
  refreshDisplay();
}

void loop() {
  #if ENABLE_ZIGBEE_START
  if (!g_zigbeeReady && Zigbee.connected()) {
    g_zigbeeReady = true;
    reportCurrentPageState();
    g_lastAction = "Zigbee connected";
    requestDisplayRefresh(10);
  }

  if (!Zigbee.connected() && millis() - g_lastZigbeeRetryMs > 5000) {
    g_lastZigbeeRetryMs = millis();
    Serial.println("Zigbee offline");
    if (!g_zigbeeReady) {
      g_lastAction = "No Zigbee";
      requestDisplayRefresh(10);
    }
  }
  #endif

  pollIqs550();
  pollMcpState();
  updateFactoryResetCombo();
  updateButtons();

  if (g_displayRefreshPending &&
      millis() >= g_displayRefreshDueMs) {
    g_displayRefreshPending = false;
    refreshDisplay();
  }

  static unsigned long lastBatteryReportMs = 0;
  if (g_zigbeeReady && millis() - lastBatteryReportMs > 60000) {
    lastBatteryReportMs = millis();
    g_remoteEndpoint.setBatteryPercentage(100);
    g_remoteEndpoint.reportBatteryPercentage();
  }

  delay(1);
}
