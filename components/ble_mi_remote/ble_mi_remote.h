#pragma once

#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/button/button.h"
#include "esphome/components/esp32_ble/esp32_ble.h"

#include <esp_gatts_api.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_bt_defs.h>

#include <string>

namespace esphome {
namespace ble_mi_remote {

// ─── Button type indices (must match __init__.py BUTTONS dict) ────────────────
static constexpr uint8_t BTN_POWER        = 0;
static constexpr uint8_t BTN_HOME         = 1;
static constexpr uint8_t BTN_BACK         = 2;
static constexpr uint8_t BTN_MENU         = 3;
static constexpr uint8_t BTN_UP           = 4;
static constexpr uint8_t BTN_DOWN         = 5;
static constexpr uint8_t BTN_LEFT         = 6;
static constexpr uint8_t BTN_RIGHT        = 7;
static constexpr uint8_t BTN_OK           = 8;
static constexpr uint8_t BTN_VOLUME_UP    = 9;
static constexpr uint8_t BTN_VOLUME_DOWN  = 10;
static constexpr uint8_t BTN_MUTE         = 11;
static constexpr uint8_t BTN_PLAY_PAUSE   = 12;
static constexpr uint8_t BTN_FAST_FORWARD = 13;
static constexpr uint8_t BTN_REWIND       = 14;
static constexpr uint8_t BTN_COUNT        = 15;

// ─── Report IDs ───────────────────────────────────────────────────────────────
static constexpr uint8_t REPORT_ID_CONSUMER  = 1;
static constexpr uint8_t REPORT_ID_KEYBOARD  = 2;

// ─── HID service attribute indices ───────────────────────────────────────────
enum HidAttrIdx : uint16_t {
  IDX_HID_SVC = 0,
  IDX_HID_CHAR_INFO_DECL,
  IDX_HID_CHAR_INFO_VAL,
  IDX_HID_CHAR_REPORT_MAP_DECL,
  IDX_HID_CHAR_REPORT_MAP_VAL,
  IDX_HID_CHAR_CTRL_DECL,
  IDX_HID_CHAR_CTRL_VAL,
  IDX_HID_CHAR_PROTO_DECL,
  IDX_HID_CHAR_PROTO_VAL,
  // Consumer Control report
  IDX_HID_CONSUMER_DECL,
  IDX_HID_CONSUMER_VAL,
  IDX_HID_CONSUMER_REF,
  IDX_HID_CONSUMER_CCCD,
  // Keyboard report
  IDX_HID_KB_DECL,
  IDX_HID_KB_VAL,
  IDX_HID_KB_REF,
  IDX_HID_KB_CCCD,
  IDX_HID_MAX,
};

// ─── Battery service attribute indices ───────────────────────────────────────
enum BasAttrIdx : uint16_t {
  IDX_BAS_SVC = 0,
  IDX_BAS_CHAR_LEVEL_DECL,
  IDX_BAS_CHAR_LEVEL_VAL,
  IDX_BAS_CHAR_LEVEL_CCCD,
  IDX_BAS_MAX,
};

// ─── Device Information service attribute indices ─────────────────────────────
enum DisAttrIdx : uint16_t {
  IDX_DIS_SVC = 0,
  IDX_DIS_CHAR_MFNAME_DECL,
  IDX_DIS_CHAR_MFNAME_VAL,
  IDX_DIS_CHAR_PNP_DECL,
  IDX_DIS_CHAR_PNP_VAL,
  IDX_DIS_MAX,
};

// ─── srvc_inst_id constants for create_attr_tab ───────────────────────────────
static constexpr uint8_t INST_DIS = 0;
static constexpr uint8_t INST_BAS = 1;
static constexpr uint8_t INST_HID = 2;

// ─── Forward declarations ─────────────────────────────────────────────────────
class BLEMiRemote;

// ─── Button entity ────────────────────────────────────────────────────────────
class MiRemoteButton : public button::Button {
 public:
  void set_parent(BLEMiRemote *parent, uint8_t type) {
    parent_ = parent;
    type_ = type;
  }

 protected:
  void press_action() override;
  BLEMiRemote *parent_{nullptr};
  uint8_t type_{0};
};

// ─── Main component ───────────────────────────────────────────────────────────
class BLEMiRemote : public Component, public esp32_ble::GAPEventHandler {
 public:
  // ── Component lifecycle ──────────────────────────────────────────────────
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    // Must run after ESP32BLE (BLUETOOTH priority = 0.0)
    return setup_priority::BLUETOOTH - 10.0f;
  }

  // ── Configuration setters (called from codegen) ──────────────────────────
  void set_device_name(const std::string &name) { device_name_ = name; }
  void set_manufacturer_name(const std::string &name) { manufacturer_name_ = name; }
  void set_battery_level(uint8_t level) { battery_level_ = level; }
  void set_reconnect(bool reconnect) { reconnect_ = reconnect; }

  void set_power_button(MiRemoteButton *b)        { register_button_(b, BTN_POWER); }
  void set_home_button(MiRemoteButton *b)         { register_button_(b, BTN_HOME); }
  void set_back_button(MiRemoteButton *b)         { register_button_(b, BTN_BACK); }
  void set_menu_button(MiRemoteButton *b)         { register_button_(b, BTN_MENU); }
  void set_up_button(MiRemoteButton *b)           { register_button_(b, BTN_UP); }
  void set_down_button(MiRemoteButton *b)         { register_button_(b, BTN_DOWN); }
  void set_left_button(MiRemoteButton *b)         { register_button_(b, BTN_LEFT); }
  void set_right_button(MiRemoteButton *b)        { register_button_(b, BTN_RIGHT); }
  void set_ok_button(MiRemoteButton *b)           { register_button_(b, BTN_OK); }
  void set_volume_up_button(MiRemoteButton *b)    { register_button_(b, BTN_VOLUME_UP); }
  void set_volume_down_button(MiRemoteButton *b)  { register_button_(b, BTN_VOLUME_DOWN); }
  void set_mute_button(MiRemoteButton *b)         { register_button_(b, BTN_MUTE); }
  void set_play_pause_button(MiRemoteButton *b)   { register_button_(b, BTN_PLAY_PAUSE); }
  void set_fast_forward_button(MiRemoteButton *b) { register_button_(b, BTN_FAST_FORWARD); }
  void set_rewind_button(MiRemoteButton *b)       { register_button_(b, BTN_REWIND); }

  // ── Called by MiRemoteButton::press_action() ─────────────────────────────
  void handle_button_press(uint8_t type);

  // ── GAPEventHandler interface ────────────────────────────────────────────
  void gap_event_handler(esp_gap_ble_cb_event_t event,
                          esp_ble_gap_cb_param_t *param) override;

  static BLEMiRemote *global_instance;

 private:
  enum class State {
    IDLE,             // waiting for BT stack
    REGISTERING,      // esp_ble_gatts_app_register sent
    CREATING,         // create_attr_tab calls in flight
    ADV_CONFIGURING,  // esp_ble_gap_config_adv_data sent
    ADVERTISING,      // advertising, waiting for connection
    CONNECTED,        // device connected
  };

  void register_button_(MiRemoteButton *b, uint8_t type);
  bool try_init_();
  void create_services_();
  void configure_adv_();
  void start_advertising_();
  void send_consumer_report_(uint16_t key);
  void send_keyboard_report_(uint8_t key);
  void release_keys_();

  static void gatts_cb_static_(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);
  void gatts_event_(esp_gatts_cb_event_t event,
                    esp_gatt_if_t gatts_if,
                    esp_ble_gatts_cb_param_t *param);

  // ── Configuration ────────────────────────────────────────────────────────
  std::string device_name_{"MI Remote"};
  std::string manufacturer_name_{"Xiaomi"};
  uint8_t battery_level_{100};
  bool reconnect_{true};

  // ── Runtime state ────────────────────────────────────────────────────────
  State state_{State::IDLE};
  uint32_t last_retry_{0};
  uint8_t services_created_{0};
  bool callback_registered_{false};
  bool app_register_pending_{false};

  esp_gatt_if_t gatts_if_{ESP_GATT_IF_NONE};
  uint16_t conn_id_{0xFFFF};
  esp_bd_addr_t remote_bda_{};

  // ── GATT attribute handles ───────────────────────────────────────────────
  uint16_t dis_handles_[IDX_DIS_MAX]{};
  uint16_t bas_handles_[IDX_BAS_MAX]{};
  uint16_t hid_handles_[IDX_HID_MAX]{};

  // ── CCCD notification flags ──────────────────────────────────────────────
  uint16_t consumer_cccd_{0};
  uint16_t keyboard_cccd_{0};
  uint16_t battery_cccd_{0};

  // ── Key release scheduling ───────────────────────────────────────────────
  bool pending_release_{false};
  uint32_t release_at_{0};

  // ── Button registry ──────────────────────────────────────────────────────
  MiRemoteButton *buttons_[BTN_COUNT]{};
};

}  // namespace ble_mi_remote
}  // namespace esphome

#endif  // USE_ESP32
