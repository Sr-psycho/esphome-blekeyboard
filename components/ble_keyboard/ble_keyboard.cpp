#ifdef USE_ESP32

/* ============================================================================
 * MAINTENANCE NOTES — READ BEFORE EDITING
 * ЗАМЕТКИ ДЛЯ ПОДДЕРЖКИ — ПРОЧИТАТЬ ПЕРЕД РЕДАКТИРОВАНИЕМ
 * ============================================================================
 *
 * EN: This file was ported from Arduino/NimBLE-Arduino to native ESP-IDF
 * (esp_hid + NimBLE host). Key facts for anyone modifying this file:
 *
 * 1. start()/stop() do a "SOFT" stop: they only stop advertising
 *    (ble_gap_adv_stop) and disconnect the current phone
 *    (ble_gap_terminate). They NEVER call nimble_port_stop()/
 *    nimble_port_deinit()/esp_hidd_dev_deinit() after setup(). A full
 *    stack teardown was tried and reliably crashed with
 *    "Guru Meditation Error: LoadProhibited" (PC 0x400828ca,
 *    EXCVADDR 0x1a) on every call, not just under a race condition.
 *    If you're tempted to "properly" tear down the stack — don't. It
 *    is not safe to call those functions from outside the NimBLE host
 *    task in this esp_hid/NimBLE combination.
 *
 * 2. "Advertise-on-demand": press()/press(MediaKeyReport) work even
 *    when the phone isn't connected. They auto-start() the stack and
 *    queue the single most recent action, which fires ~600ms after
 *    ESP_HIDD_CONNECT_EVENT (the phone needs time to subscribe to HID
 *    notifications first — this delay was raised from 300ms to 600ms
 *    after field testing showed some media player apps need longer
 *    before the very first report after connect is reliably delivered).
 *
 * 3. Auto-release: every press we send is ALWAYS followed by our own
 *    zero (release) report ~120ms later, regardless of what any
 *    external YAML automation does. This is required because HID
 *    Consumer Control codes like Next/Previous Track (0xB5/0xB6) and
 *    Volume Up/Down (0xE9/0xEA) are treated by many phones as
 *    "hold to keep going" — an un-released press causes seeking
 *    instead of track-skip, and runaway volume changes.
 *
 * 4. Auto-idle-disconnect: 30 seconds after the last report sent,
 *    stop() is called automatically. This replaces any external
 *    Home Assistant script/automation that used to do the same thing
 *    — do NOT add such a script back, it will race with this logic.
 *
 * RU: Этот файл был портирован с Arduino/NimBLE-Arduino на нативный
 * ESP-IDF (esp_hid + NimBLE host). Ключевые факты для тех, кто будет
 * это редактировать:
 *
 * 1. start()/stop() делают "МЯГКИЙ" стоп: только останавливают
 *    адвертайзинг (ble_gap_adv_stop) и разрывают текущее соединение с
 *    телефоном (ble_gap_terminate). Они НИКОГДА не вызывают
 *    nimble_port_stop()/nimble_port_deinit()/esp_hidd_dev_deinit()
 *    после setup(). Полный снос стека был опробован и стабильно
 *    приводил к крэшу "Guru Meditation Error: LoadProhibited"
 *    (PC 0x400828ca, EXCVADDR 0x1a) при КАЖДОМ вызове, а не только
 *    при гонке. Если возникнет соблазн сделать "правильный" полный
 *    снос — не делайте. В данной связке esp_hid/NimBLE вызывать эти
 *    функции не из host-задачи NimBLE небезопасно.
 *
 * 2. "Адвертайзинг по требованию": press()/press(MediaKeyReport)
 *    работают, даже если телефон не подключён. Они сами вызывают
 *    start() и ставят в очередь одну (последнюю) команду, которая
 *    отправляется через ~600мс после ESP_HIDD_CONNECT_EVENT (телефону
 *    нужно время подписаться на HID-нотификации — эта задержка была
 *    увеличена с 300мс до 600мс после тестирования: некоторым
 *    приложениям-плеерам требуется больше времени, прежде чем первый
 *    отчёт после подключения гарантированно доставляется).
 *
 * 3. Авто-отпускание: за каждым отправленным нажатием ВСЕГДА следует
 *    наш собственный нулевой (release) отчёт через ~120мс, независимо
 *    от того, что делает внешняя YAML-автоматизация. Это обязательно,
 *    так как HID Consumer Control коды вроде Next/Previous Track
 *    (0xB5/0xB6) и Volume Up/Down (0xE9/0xEA) многими телефонами
 *    трактуются как "держать = продолжать" — неотпущенное нажатие
 *    вызывает перемотку вместо переключения трека и улетающую громкость.
 *
 * 4. Авто-отключение по бездействию: через 30 секунд после последней
 *    отправленной команды stop() вызывается автоматически. Это
 *    заменяет любой внешний скрипт/автоматизацию в Home Assistant,
 *    которая раньше делала то же самое — НЕ возвращайте такой скрипт,
 *    он будет конфликтовать с этой логикой.
 * ============================================================================
 */

#include "ble_keyboard.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cctype>
#include <cstdlib>
#include <vector>
#include <mutex>
#include <atomic>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_nimble_mem.h"
#include "esp_bt.h"

#include "esp_hid_common.h"
#include "esp_hidd.h"
#include "esp_hid_gap.h"

extern "C" uint16_t esp_hid_ble_gap_conn_handle(void);
extern "C" esp_err_t esp_hid_ble_gap_adv_stop(void);

namespace esphome {
namespace ble_keyboard {

static const char *const TAG = "ble_keyboard";

/* --- HID Report Map --- */
static const uint8_t kHidReportMap[] = {
    // Keyboard (Report ID 1)
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,
    0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x95, 0x05, 0x75, 0x01, 0x05, 0x08,
    0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00,
    0x25, 0xFF, 0x05, 0x07, 0x19, 0x00,
    0x29, 0xFF, 0x81, 0x00,
    0xC0,
    // Consumer Control (Report ID 2)
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01,
    0x85, 0x02,
    0x19, 0x00, 0x2A, 0x3C, 0x02,
    0x15, 0x00, 0x26, 0xFF, 0x03,
    0x95, 0x01, 0x75, 0x10, 0x81, 0x00,
    0xC0,
};

/* --- Module state --- */
static bool g_connected = false;
static bool g_reconnect = true;
static uint8_t keyboard_output_report_[1] = {0};
static char g_device_name[32] = "BLE Keyboard";
static char g_manufacturer_id[32] = "ESPHome";

static std::mutex s_lifecycle_mtx;
static std::atomic<bool> s_stack_running{false};

static esp_hidd_dev_t *s_hid_dev = NULL;
static int s_advertising_startup_delay = 0;

static Esp32BleKeyboard *s_instance = nullptr;

/* ========================================================================
 * ADVERTISE-ON-DEMAND + AUTO-IDLE-DISCONNECT + AUTO-RELEASE
 * АДВЕРТАЙЗИНГ-ПО-ТРЕБОВАНИЮ + АВТООТКЛЮЧЕНИЕ ПО БЕЗДЕЙСТВИЮ + АВТООТПУСКАНИЕ
 * ======================================================================== */
static constexpr uint32_t kAutoIdleDisconnectMs = 30000;   // 30s idle timeout
static constexpr uint32_t kPostConnectSettleMs = 600;      // let the central subscribe to notifications (bumped from 300ms: some media players/OS combos need more time before the first HID report after connect is reliably delivered)
static constexpr uint32_t kPendingReportMaxWaitMs = 10000; // give up waiting for a connection after this
static constexpr uint32_t kAutoReleaseMs = 120;            // press->release gap, matches a natural "tap"

enum class PendingReportKind : uint8_t { NONE = 0, KEYBOARD = 1, MEDIA = 2 };
static PendingReportKind s_pending_kind = PendingReportKind::NONE;
static uint8_t s_pending_buf[8] = {0};
static size_t s_pending_len = 0;

static void schedule_idle_disconnect() {
  if (s_instance == nullptr) {
    return;
  }
  App.scheduler.cancel_timeout(s_instance, "ble_idle_disconnect");
  App.scheduler.set_timeout(s_instance, "ble_idle_disconnect", kAutoIdleDisconnectMs, []() {
    ESP_LOGI(TAG, "Auto-disconnect: %u ms of inactivity elapsed", (unsigned) kAutoIdleDisconnectMs);
    if (s_instance != nullptr) {
      s_instance->stop();
    }
  });
}

static void schedule_auto_release(uint8_t report_id, size_t len) {
  if (s_instance == nullptr) {
    return;
  }
  App.scheduler.cancel_timeout(s_instance, "ble_auto_release");
  App.scheduler.set_timeout(s_instance, "ble_auto_release", kAutoReleaseMs, [report_id, len]() {
    if (s_hid_dev != nullptr) {
      uint8_t zero[8] = {0};
      esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 0, report_id, zero, len);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hidd_dev_input_set (auto-release) failed: %d", err);
      } else {
        ESP_LOGD(TAG, "Auto-release sent for report_id=%d", report_id);
      }
    }
    schedule_idle_disconnect();
  });
}

static void flush_pending_report() {
  if (s_pending_kind == PendingReportKind::NONE || s_hid_dev == nullptr) {
    return;
  }
  uint8_t report_id = (s_pending_kind == PendingReportKind::KEYBOARD) ? 1 : 2;
  size_t len = s_pending_len;
  esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 0, report_id, s_pending_buf, len);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hidd_dev_input_set (flush queued action) failed: %d", err);
  } else {
    ESP_LOGI(TAG, "Sent queued action after connect (report_id=%d)", report_id);
    schedule_auto_release(report_id, len);
  }
  s_pending_kind = PendingReportKind::NONE;
}

static void queue_or_start(PendingReportKind kind, const uint8_t *buf, size_t len) {
  memcpy(s_pending_buf, buf, len);
  s_pending_len = len;
  s_pending_kind = kind;

  if (!s_stack_running.load()) {
    ESP_LOGI(TAG, "Advertising on demand: starting BLE stack for queued action");
    if (s_instance != nullptr) {
      s_instance->start();
    }
  } else {
    ESP_LOGI(TAG, "Not connected yet; action queued, waiting for phone to connect");
  }

  if (s_instance != nullptr) {
    App.scheduler.cancel_timeout(s_instance, "ble_pending_timeout");
    App.scheduler.set_timeout(s_instance, "ble_pending_timeout", kPendingReportMaxWaitMs, []() {
      if (s_pending_kind != PendingReportKind::NONE) {
        ESP_LOGW(TAG, "Queued action timed out waiting for a connection (%u ms); discarding",
                 (unsigned) kPendingReportMaxWaitMs);
        s_pending_kind = PendingReportKind::NONE;
      }
    });
  }
}

extern "C" void ble_store_config_init(void);

extern "C" void nimble_host_task(void *param) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static void hidd_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  esp_hidd_event_t event = (esp_hidd_event_t) id;
  esp_hidd_event_data_t *param = (esp_hidd_event_data_t *) event_data;

  switch (event) {
    case ESP_HIDD_START_EVENT:
      ESP_LOGI(TAG, "HID device stack started; advertising will begin in 2s");
      s_advertising_startup_delay = 2;
      break;
    case ESP_HIDD_CONNECT_EVENT:
      g_connected = true;
      ESP_LOGI(TAG, "HID device connected");
      if (s_pending_kind != PendingReportKind::NONE && s_instance != nullptr) {
        App.scheduler.cancel_timeout(s_instance, "ble_pending_timeout");
        App.scheduler.set_timeout(s_instance, "ble_pending_flush", kPostConnectSettleMs, []() { flush_pending_report(); });
      } else {
        schedule_idle_disconnect();
      }
      break;
    case ESP_HIDD_DISCONNECT_EVENT:
      g_connected = false;
      ESP_LOGI(TAG, "HID device disconnected; reason=%d", param->disconnect.reason);
      if (s_instance != nullptr) {
        App.scheduler.cancel_timeout(s_instance, "ble_idle_disconnect");
        App.scheduler.cancel_timeout(s_instance, "ble_auto_release");
      }
      if (g_reconnect && s_stack_running.load()) {
        esp_hid_ble_gap_adv_start();
      }
      break;
    case ESP_HIDD_OUTPUT_EVENT:
      if (param->output.length > 0 && param->output.data != nullptr) {
        memcpy(keyboard_output_report_, param->output.data,
               param->output.length < sizeof(keyboard_output_report_)
                   ? param->output.length
                   : sizeof(keyboard_output_report_));
        ESP_LOGI(TAG, "Keyboard LED output: 0x%02X", keyboard_output_report_[0]);
      }
      break;
    default:
      break;
  }
}

void Esp32BleKeyboard::setup() {
  ESP_LOGI(TAG, "Setting up BLE Keyboard (esp_hid device API)");
  s_instance = this;

  strncpy(g_device_name, name_.c_str(), sizeof(g_device_name) - 1);
  g_device_name[sizeof(g_device_name) - 1] = '\0';
  strncpy(g_manufacturer_id, manufacturer_id_.c_str(), sizeof(g_manufacturer_id) - 1);
  g_manufacturer_id[sizeof(g_manufacturer_id) - 1] = '\0';

  esp_err_t err = esp_hid_gap_init(ESP_HID_TRANSPORT_BLE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hid_gap_init failed: %d", err);
    return;
  }

  err = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_KEYBOARD, g_device_name);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hid_ble_gap_adv_init failed: %d", err);
    return;
  }

  ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

  static esp_hid_raw_report_map_t report_maps[] = {
      { .data = kHidReportMap, .len = sizeof(kHidReportMap) },
  };

  esp_hid_device_config_t hid_config = {
      .vendor_id          = 0x1234,
      .product_id         = 0x0001,
      .version            = 0x0001,
      .device_name        = g_device_name,
      .manufacturer_name  = g_manufacturer_id,
      .serial_number      = "1234567890",
      .report_maps        = report_maps,
      .report_maps_len    = 1,
  };

  err = esp_hidd_dev_init(&hid_config, ESP_HID_TRANSPORT_BLE, hidd_event_handler, &s_hid_dev);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hidd_dev_init failed: %d", err);
    return;
  }

  ble_svc_gap_device_name_set(g_device_name);
  esp_hidd_dev_battery_set(s_hid_dev, battery_level_);

  g_reconnect = reconnect_;

  ble_store_config_init();
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  err = esp_nimble_enable(reinterpret_cast<void *>(nimble_host_task));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_nimble_enable failed: %d", err);
    return;
  }
  s_stack_running = true;
  ESP_LOGI(TAG, "NimBLE host task started; advertising will begin on host sync");
}

void Esp32BleKeyboard::update() {
  if (state_sensor_ != nullptr) {
    state_sensor_->publish_state(g_connected);
  }
  if (s_advertising_startup_delay > 0) {
    s_advertising_startup_delay--;
    if (s_advertising_startup_delay == 0) {
      ESP_LOGI(TAG, "Advertising startup delay elapsed; starting now");
      esp_hid_ble_gap_adv_start();
    }
  }
}

void Esp32BleKeyboard::set_battery_level(uint8_t level) {
  battery_level_ = level;
  if (s_hid_dev != nullptr) {
    esp_hidd_dev_battery_set(s_hid_dev, level);
  }
}

void Esp32BleKeyboard::send_keyboard_report(uint8_t modifiers, uint8_t key1, uint8_t key2,
                                             uint8_t key3, uint8_t key4, uint8_t key5, uint8_t key6) {
  uint8_t buffer[8] = { modifiers, 0, key1, key2, key3, key4, key5, key6 };
  bool is_press = (modifiers != 0) || (key1 != 0) || (key2 != 0) || (key3 != 0) ||
                  (key4 != 0) || (key5 != 0) || (key6 != 0);

  if (!g_connected) {
    if (is_press) {
      queue_or_start(PendingReportKind::KEYBOARD, buffer, sizeof(buffer));
    }
    return;
  }
  if (s_hid_dev == nullptr) {
    return;
  }
  esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 0, 1, buffer, sizeof(buffer));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hidd_dev_input_set (keyboard) failed: %d", err);
    return;
  }
  if (is_press) {
    schedule_auto_release(1, sizeof(buffer));
  } else {
    schedule_idle_disconnect();
  }
}

void Esp32BleKeyboard::send_media_report(uint8_t byte0, uint8_t byte1) {
  uint8_t buffer[2] = { byte0, byte1 };
  bool is_press = (byte0 != 0) || (byte1 != 0);

  if (!g_connected) {
    if (is_press) {
      queue_or_start(PendingReportKind::MEDIA, buffer, sizeof(buffer));
    }
    return;
  }
  if (s_hid_dev == nullptr) {
    return;
  }
  esp_err_t err = esp_hidd_dev_input_set(s_hid_dev, 0, 2, buffer, sizeof(buffer));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hidd_dev_input_set (media) failed: %d", err);
    return;
  }
  if (is_press) {
    schedule_auto_release(2, sizeof(buffer));
  } else {
    schedule_idle_disconnect();
  }
}

struct HidKey {
  uint8_t modifier;
  uint8_t key;
};

static HidKey ascii_to_hid(char c) {
  if (c >= 'a' && c <= 'z') return {0, (uint8_t)(0x04 + (c - 'a'))};
  if (c >= 'A' && c <= 'Z') return {0x02, (uint8_t)(0x04 + (c - 'A'))};
  if (c >= '1' && c <= '9') return {0, (uint8_t)(0x1E + (c - '1'))};
  if (c == '0') return {0, 0x27};
  if (c == '\n') return {0, 0x28};
  if (c == '\r') return {0, 0x28};
  if (c == '\x1B') return {0, 0x29};
  if (c == '\b') return {0, 0x2A};
  if (c == '\t') return {0, 0x2B};
  if (c == ' ') return {0, 0x2C};
  if (c == '-') return {0, 0x2D};
  if (c == '_') return {0x02, 0x2D};
  if (c == '=') return {0, 0x2E};
  if (c == '+') return {0x02, 0x2E};
  if (c == '[') return {0, 0x2F};
  if (c == '{') return {0x02, 0x2F};
  if (c == ']') return {0, 0x30};
  if (c == '}') return {0x02, 0x30};
  if (c == '\\') return {0, 0x31};
  if (c == '|') return {0x02, 0x31};
  if (c == ';') return {0, 0x33};
  if (c == ':') return {0x02, 0x33};
  if (c == '\'') return {0, 0x34};
  if (c == '"') return {0x02, 0x34};
  if (c == '`') return {0, 0x35};
  if (c == '~') return {0x02, 0x35};
  if (c == ',') return {0, 0x36};
  if (c == '<') return {0x02, 0x36};
  if (c == '.') return {0, 0x37};
  if (c == '>') return {0x02, 0x37};
  if (c == '/') return {0, 0x38};
  if (c == '?') return {0x02, 0x38};
  if (c == '!') return {0x02, 0x1E};
  if (c == '@') return {0x02, 0x1F};
  if (c == '#') return {0x02, 0x20};
  if (c == '$') return {0x02, 0x21};
  if (c == '%') return {0x02, 0x22};
  if (c == '^') return {0x02, 0x23};
  if (c == '&') return {0x02, 0x24};
  if (c == '*') return {0x02, 0x25};
  if (c == '(') return {0x02, 0x26};
  if (c == ')') return {0x02, 0x27};
  return {0, 0};
}

void Esp32BleKeyboard::press(std::string message) {
  if (!g_connected) {
    ESP_LOGW(TAG, "Not connected, cannot print");
    return;
  }
  if (pending_text_.length() > 0) {
    ESP_LOGW(TAG, "Already printing; replacing with new message");
    pending_text_ = message;
    text_index_ = 0;
    return;
  }
  pending_text_ = message;
  text_index_ = 0;
  process_next_print_char();
}

void Esp32BleKeyboard::process_next_print_char() {
  if (pending_text_.empty() || text_index_ >= pending_text_.length()) {
    pending_text_.clear();
    text_index_ = 0;
    return;
  }

  HidKey hk = ascii_to_hid(pending_text_[text_index_]);
  text_index_++;

  if (hk.key == 0) {
    ESP_LOGW(TAG, "Unsupported character: 0x%02X", (unsigned char)pending_text_[text_index_ - 1]);
    set_timeout("print_next", 1, [this]() { this->process_next_print_char(); });
    return;
  }

  send_keyboard_report(hk.modifier, hk.key);
  set_timeout("print_release", default_delay_, [this]() {
    this->send_keyboard_report(0, 0);
    this->set_timeout("print_next", this->default_delay_, [this]() {
      this->process_next_print_char();
    });
  });
}

static bool is_modifier(uint8_t key) {
  switch (key) {
    case 0x01: case 0x02: case 0x04: case 0x08:
    case 0x10: case 0x20: case 0x40: case 0x80:
      return true;
    default:
      return false;
  }
}

void Esp32BleKeyboard::press(uint8_t key, bool with_timer) {
  if (with_timer) {
    update_timer();
  }
  if (is_modifier(key)) {
    send_keyboard_report(key, 0);
  } else {
    send_keyboard_report(0, key);
  }
}

void Esp32BleKeyboard::press(MediaKeyReport key, bool with_timer) {
  if (with_timer) {
    update_timer();
  }
  send_media_report(key[0], key[1]);
}

void Esp32BleKeyboard::press_combination(const std::vector<std::string> &keys, uint32_t hold_ms) {
  if (!g_connected) {
    ESP_LOGW(TAG, "Not connected, cannot press combination");
    return;
  }

  uint8_t modifiers = 0;
  uint8_t keycodes[6] = {0};
  size_t key_idx = 0;

  for (const std::string &key_str : keys) {
    uint8_t key_val = 0;
    bool is_str_key = true;

    if (key_str.length() == 1 && !std::isdigit(static_cast<unsigned char>(key_str[0]))) {
      HidKey hk = ascii_to_hid(key_str[0]);
      if (hk.key != 0) {
        key_val = hk.key;
        modifiers |= hk.modifier;
      }
    } else if (!key_str.empty() && std::isdigit(static_cast<unsigned char>(key_str[0]))) {
      key_val = static_cast<uint8_t>(strtol(key_str.c_str(), nullptr, 0));
      is_str_key = false;
    }

    if (key_val == 0) {
      continue;
    }

    if (!is_str_key && is_modifier(key_val)) {
      modifiers |= key_val;
    } else if (key_idx < 6) {
      keycodes[key_idx++] = key_val;
    }
  }

  send_keyboard_report(modifiers, keycodes[0], keycodes[1], keycodes[2],
                       keycodes[3], keycodes[4], keycodes[5]);

  if (hold_ms > 0) {
    cancel_timeout(TAG);
    set_timeout(TAG, hold_ms, [this]() { this->release(); });
  }
}

void Esp32BleKeyboard::release() {
  if (!g_connected) {
    return;
  }
  cancel_timeout(TAG);
  send_keyboard_report(0, 0);
  send_media_report(0, 0);
}

/* ========================================================================
 * SOFT START/STOP — disconnect + toggle advertising only.
 * МЯГКИЙ ЗАПУСК/ОСТАНОВ — только разрыв соединения + переключение
 * адвертайзинга. NimBLE-хост НИКОГДА не разрушается после setup().
 * ======================================================================== */

void Esp32BleKeyboard::start() {
  std::lock_guard<std::mutex> lock(s_lifecycle_mtx);

  if (s_stack_running) {
    ESP_LOGW(TAG, "BLE keyboard already active, ignoring start()");
    return;
  }

  ESP_LOGI(TAG, "Starting BLE keyboard (resuming advertising)");
  s_stack_running = true;

  esp_err_t err = esp_hid_ble_gap_adv_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_hid_ble_gap_adv_start failed: %d", err);
  }
}

void Esp32BleKeyboard::stop() {
  std::lock_guard<std::mutex> lock(s_lifecycle_mtx);

  if (!s_stack_running) {
    ESP_LOGW(TAG, "BLE keyboard already stopped, ignoring stop()");
    return;
  }

  ESP_LOGI(TAG, "Stopping BLE keyboard (stopping advertising + forcing disconnect)");

  s_stack_running = false;

  App.scheduler.cancel_timeout(this, "ble_idle_disconnect");
  App.scheduler.cancel_timeout(this, "ble_pending_timeout");
  App.scheduler.cancel_timeout(this, "ble_pending_flush");
  App.scheduler.cancel_timeout(this, "ble_auto_release");
  s_pending_kind = PendingReportKind::NONE;

  esp_hid_ble_gap_adv_stop();

  uint16_t conn_handle = esp_hid_ble_gap_conn_handle();
  if (conn_handle != 0xffff /* BLE_HS_CONN_HANDLE_NONE */) {
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
      ESP_LOGW(TAG, "ble_gap_terminate failed: %d (device may still show as connected)", rc);
    }
  }

  g_connected = false;
  ESP_LOGI(TAG, "BLE keyboard stopped; not advertising, any active connection was terminated");
}

bool Esp32BleKeyboard::is_connected() {
  return g_connected;
}

void Esp32BleKeyboard::update_timer() {
  cancel_timeout(TAG);
  set_timeout(TAG, release_delay_, [this]() { this->release(); });
}

}  // namespace ble_keyboard
}  // namespace esphome

#endif  // USE_ESP32
