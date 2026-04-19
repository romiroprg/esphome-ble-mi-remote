#ifdef USE_ESP32

#include "ble_mi_remote.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>

namespace esphome {
namespace ble_mi_remote {

static const char *const TAG = "ble_mi_remote";

// ─────────────────────────────────────────────────────────────────────────────
// HID Report Descriptor
// Report ID 1: Consumer Control (media keys, home, back, power …)
// Report ID 2: Keyboard        (arrow keys, enter)
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t HID_REPORT_MAP[] = {
    // ── Consumer Control (Report ID 1) ──────────────────────────────────────
    0x05, 0x0C,        // Usage Page (Consumer Devices)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x75, 0x10,        //   Report Size (16 bits)
    0x95, 0x01,        //   Report Count (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x03,  //   Logical Maximum (1023)
    0x19, 0x00,        //   Usage Minimum (0)
    0x2A, 0xFF, 0x03,  //   Usage Maximum (1023)
    0x81, 0x00,        //   Input (Data, Array, Absolute)
    0xC0,              // End Collection

    // ── Keyboard (Report ID 2) ───────────────────────────────────────────────
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    // Modifier byte
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        //   Usage Minimum (Left Control)
    0x29, 0xE7,        //   Usage Maximum (Right GUI)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    // Reserved byte
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Constant)
    // Keycodes (6 bytes)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x06,        //   Report Count (6)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,        //   Usage Minimum (Reserved)
    0x29, 0x65,        //   Usage Maximum (Keyboard Application)
    0x81, 0x00,        //   Input (Data, Array, Absolute)
    0xC0,              // End Collection
};

// ─────────────────────────────────────────────────────────────────────────────
// Consumer Control keycodes (HID Usage Page 0x0C)
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t CONSUMER_KEY[BTN_COUNT] = {
    /* POWER        */ 0x0030,
    /* HOME         */ 0x0223,
    /* BACK         */ 0x0224,
    /* MENU         */ 0x0040,
    /* UP           */ 0x0000,  // keyboard
    /* DOWN         */ 0x0000,
    /* LEFT         */ 0x0000,
    /* RIGHT        */ 0x0000,
    /* OK           */ 0x0000,
    /* VOLUME_UP    */ 0x00E9,
    /* VOLUME_DOWN  */ 0x00EA,
    /* MUTE         */ 0x00E2,
    /* PLAY_PAUSE   */ 0x00CD,
    /* FAST_FORWARD */ 0x00B3,
    /* REWIND       */ 0x00B4,
};

// Keyboard HID keycodes (USB HID Usage Page 0x07)
static const uint8_t KEYBOARD_KEY[BTN_COUNT] = {
    /* POWER        */ 0x00,
    /* HOME         */ 0x00,
    /* BACK         */ 0x00,
    /* MENU         */ 0x00,
    /* UP           */ 0x52,  // Right Arrow
    /* DOWN         */ 0x51,
    /* LEFT         */ 0x50,
    /* RIGHT        */ 0x4F,
    /* OK           */ 0x28,  // Enter
    /* VOLUME_UP    */ 0x00,
    /* VOLUME_DOWN  */ 0x00,
    /* MUTE         */ 0x00,
    /* PLAY_PAUSE   */ 0x00,
    /* FAST_FORWARD */ 0x00,
    /* REWIND       */ 0x00,
};

// ─────────────────────────────────────────────────────────────────────────────
// Static characteristic values
// ─────────────────────────────────────────────────────────────────────────────
// HID Information: bcdHID=1.11, bCountryCode=0, Flags=normallyConnectable
static const uint8_t HID_INFO_VAL[]    = {0x11, 0x01, 0x00, 0x02};
static uint8_t       PROTO_MODE_VAL[]  = {0x01};   // Report protocol
static uint8_t       CTRL_POINT_VAL[]  = {0x00};
// PnP ID: VendorSrc=Bluetooth SIG, VID=0x2717 (Xiaomi), PID=0x32B1, Ver=0x0001
static const uint8_t PNP_ID_VAL[]     = {0x01, 0x17, 0x27, 0xB1, 0x32, 0x01, 0x00};
// Report Reference descriptors: {report_id, report_type=Input(1)}
static const uint8_t CONSUMER_REF[]   = {REPORT_ID_CONSUMER, 0x01};
static const uint8_t KB_REF[]         = {REPORT_ID_KEYBOARD, 0x01};
// CCCD init (notifications disabled)
static uint8_t CCCD_INIT[]            = {0x00, 0x00};
// Report value buffers (mutable, sent via notify)
static uint8_t consumer_report_buf[2] = {0x00, 0x00};
static uint8_t keyboard_report_buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t battery_level_buf[1]   = {100};

// Dynamic: manufacturer name string (filled in setup())
static char mfname_buf[64] = "Xiaomi";

// ─────────────────────────────────────────────────────────────────────────────
// UUID helpers
// ─────────────────────────────────────────────────────────────────────────────
static const uint16_t UUID_PRIMARY_SVC    = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t UUID_CHAR_DECL      = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t UUID_CCCD           = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t UUID_REPORT_REF     = 0x2908;
static const uint16_t UUID_SVC_DIS        = 0x180A;
static const uint16_t UUID_SVC_BAS        = 0x180F;
static const uint16_t UUID_SVC_HID        = 0x1812;
static const uint16_t UUID_CHAR_MFNAME    = 0x2A29;
static const uint16_t UUID_CHAR_PNP       = 0x2A50;
static const uint16_t UUID_CHAR_BATLEVEL  = 0x2A19;
static const uint16_t UUID_CHAR_HIDINFO   = 0x2A4A;
static const uint16_t UUID_CHAR_REPORTMAP = 0x2A4B;
static const uint16_t UUID_CHAR_HIDCTRL   = 0x2A4C;
static const uint16_t UUID_CHAR_PROTOMODE = 0x2A4E;
static const uint16_t UUID_CHAR_REPORT    = 0x2A4D;

static const uint8_t PROP_R  = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t PROP_RN = ESP_GATT_CHAR_PROP_BIT_READ  | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t PROP_RW = ESP_GATT_CHAR_PROP_BIT_READ  | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t PROP_W  = ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

// ─────────────────────────────────────────────────────────────────────────────
// Device Information Service attribute table
// ─────────────────────────────────────────────────────────────────────────────
static esp_gatts_attr_db_t DIS_ATTR_DB[IDX_DIS_MAX] = {
    // Service declaration
    [IDX_DIS_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRIMARY_SVC, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(UUID_SVC_DIS), (uint8_t *)&UUID_SVC_DIS}
    },
    // Manufacturer Name: declaration
    [IDX_DIS_CHAR_MFNAME_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_R), (uint8_t *)&PROP_R}
    },
    // Manufacturer Name: value  (pointer set to mfname_buf at runtime)
    [IDX_DIS_CHAR_MFNAME_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_MFNAME, ESP_GATT_PERM_READ,
         sizeof(mfname_buf), 0, (uint8_t *)mfname_buf}
    },
    // PnP ID: declaration
    [IDX_DIS_CHAR_PNP_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_R), (uint8_t *)&PROP_R}
    },
    // PnP ID: value
    [IDX_DIS_CHAR_PNP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_PNP, ESP_GATT_PERM_READ,
         sizeof(PNP_ID_VAL), sizeof(PNP_ID_VAL), (uint8_t *)PNP_ID_VAL}
    },
};

// ─────────────────────────────────────────────────────────────────────────────
// Battery Service attribute table
// ─────────────────────────────────────────────────────────────────────────────
static esp_gatts_attr_db_t BAS_ATTR_DB[IDX_BAS_MAX] = {
    [IDX_BAS_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRIMARY_SVC, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(UUID_SVC_BAS), (uint8_t *)&UUID_SVC_BAS}
    },
    [IDX_BAS_CHAR_LEVEL_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_RN), (uint8_t *)&PROP_RN}
    },
    [IDX_BAS_CHAR_LEVEL_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_BATLEVEL, ESP_GATT_PERM_READ,
         sizeof(battery_level_buf), sizeof(battery_level_buf), battery_level_buf}
    },
    [IDX_BAS_CHAR_LEVEL_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(CCCD_INIT), sizeof(CCCD_INIT), CCCD_INIT}
    },
};

// ─────────────────────────────────────────────────────────────────────────────
// HID Service attribute table
// ─────────────────────────────────────────────────────────────────────────────
static esp_gatts_attr_db_t HID_ATTR_DB[IDX_HID_MAX] = {
    [IDX_HID_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_PRIMARY_SVC, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(UUID_SVC_HID), (uint8_t *)&UUID_SVC_HID}
    },
    // HID Information
    [IDX_HID_CHAR_INFO_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_R), (uint8_t *)&PROP_R}
    },
    [IDX_HID_CHAR_INFO_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_HIDINFO, ESP_GATT_PERM_READ_ENCRYPTED,
         sizeof(HID_INFO_VAL), sizeof(HID_INFO_VAL), (uint8_t *)HID_INFO_VAL}
    },
    // Report Map
    [IDX_HID_CHAR_REPORT_MAP_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_R), (uint8_t *)&PROP_R}
    },
    [IDX_HID_CHAR_REPORT_MAP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_REPORTMAP, ESP_GATT_PERM_READ_ENCRYPTED,
         sizeof(HID_REPORT_MAP), sizeof(HID_REPORT_MAP), (uint8_t *)HID_REPORT_MAP}
    },
    // HID Control Point
    [IDX_HID_CHAR_CTRL_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_W), (uint8_t *)&PROP_W}
    },
    [IDX_HID_CHAR_CTRL_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_HIDCTRL, ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(CTRL_POINT_VAL), sizeof(CTRL_POINT_VAL), CTRL_POINT_VAL}
    },
    // Protocol Mode
    [IDX_HID_CHAR_PROTO_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_RW), (uint8_t *)&PROP_RW}
    },
    [IDX_HID_CHAR_PROTO_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_PROTOMODE,
         ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(PROTO_MODE_VAL), sizeof(PROTO_MODE_VAL), PROTO_MODE_VAL}
    },
    // Consumer Control Report
    [IDX_HID_CONSUMER_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_RN), (uint8_t *)&PROP_RN}
    },
    [IDX_HID_CONSUMER_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_REPORT,
         ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(consumer_report_buf), sizeof(consumer_report_buf), consumer_report_buf}
    },
    [IDX_HID_CONSUMER_REF] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_REPORT_REF, ESP_GATT_PERM_READ_ENCRYPTED,
         sizeof(CONSUMER_REF), sizeof(CONSUMER_REF), (uint8_t *)CONSUMER_REF}
    },
    [IDX_HID_CONSUMER_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD,
         ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(CCCD_INIT), sizeof(CCCD_INIT), CCCD_INIT}
    },
    // Keyboard Report
    [IDX_HID_KB_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_DECL, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(PROP_RN), (uint8_t *)&PROP_RN}
    },
    [IDX_HID_KB_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CHAR_REPORT,
         ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(keyboard_report_buf), sizeof(keyboard_report_buf), keyboard_report_buf}
    },
    [IDX_HID_KB_REF] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_REPORT_REF, ESP_GATT_PERM_READ_ENCRYPTED,
         sizeof(KB_REF), sizeof(KB_REF), (uint8_t *)KB_REF}
    },
    [IDX_HID_KB_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&UUID_CCCD,
         ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED,
         sizeof(CCCD_INIT), sizeof(CCCD_INIT), CCCD_INIT}
    },
};

// ─────────────────────────────────────────────────────────────────────────────
// Advertising data
// ─────────────────────────────────────────────────────────────────────────────
static uint16_t adv_svc_uuid = 0x1812;  // HID Service

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,  // 7.5 ms
    .max_interval        = 0x0010,  // 20 ms
    .appearance          = 0x0180,  // Generic Remote Control
    .manufacturer_len    = 0,
    .p_manufacturer_data = nullptr,
    .service_data_len    = 0,
    .p_service_data      = nullptr,
    .service_uuid_len    = sizeof(adv_svc_uuid),
    .p_service_uuid      = (uint8_t *)&adv_svc_uuid,
    .flag                = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x0020,  // 20 ms
    .adv_int_max       = 0x0040,  // 40 ms
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr         = {},
    .peer_addr_type    = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// ─────────────────────────────────────────────────────────────────────────────
// Singleton
// ─────────────────────────────────────────────────────────────────────────────
BLEMiRemote *BLEMiRemote::global_instance = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// MiRemoteButton
// ─────────────────────────────────────────────────────────────────────────────
void MiRemoteButton::press_action() {
    if (parent_ != nullptr) {
        parent_->handle_button_press(type_);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BLEMiRemote – setup / loop / dump_config
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::setup() {
    global_instance = this;

    // Copy config into static buffers used by attr tables
    battery_level_buf[0] = battery_level_;
    strncpy(mfname_buf, manufacturer_name_.c_str(), sizeof(mfname_buf) - 1);
    mfname_buf[sizeof(mfname_buf) - 1] = '\0';
    DIS_ATTR_DB[IDX_DIS_CHAR_MFNAME_VAL].att_desc.length = strlen(mfname_buf);
    DIS_ATTR_DB[IDX_DIS_CHAR_MFNAME_VAL].att_desc.max_length = strlen(mfname_buf);

    // Register as GAP event listener via ESPHome dispatcher.
    // Device name is set later in try_init_() once the Bluedroid stack is up.
    esp32_ble::global_esp32_ble->add_gap_event_handler(this);

    ESP_LOGI(TAG, "Setup done. GATTS init will follow in loop().");
}

void BLEMiRemote::loop() {
    // ── Release scheduled key ─────────────────────────────────────────────
    if (pending_release_ && millis() >= release_at_) {
        pending_release_ = false;
        release_keys_();
    }

    // ── State machine ─────────────────────────────────────────────────────
    if (state_ == State::IDLE) {
        if (millis() - last_retry_ < 2000) return;
        last_retry_ = millis();

        if (try_init_()) {
            state_ = State::REGISTERING;
        }
    }
}

void BLEMiRemote::dump_config() {
    ESP_LOGCONFIG(TAG, "BLE MI Remote:");
    ESP_LOGCONFIG(TAG, "  Name: %s", device_name_.c_str());
    ESP_LOGCONFIG(TAG, "  Manufacturer: %s", manufacturer_name_.c_str());
    ESP_LOGCONFIG(TAG, "  Battery Level: %d%%", battery_level_);
    ESP_LOGCONFIG(TAG, "  Auto-reconnect: %s", reconnect_ ? "yes" : "no");
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::register_button_(MiRemoteButton *b, uint8_t type) {
    if (type < BTN_COUNT) {
        buttons_[type] = b;
        b->set_parent(this, type);
    }
}

bool BLEMiRemote::try_init_() {
    // Register GATTS callback once. Safe to skip on retry.
    if (!callback_registered_) {
        esp_err_t ret = esp_ble_gatts_register_callback(gatts_cb_static_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "gatts_register_callback failed (%s), will retry",
                     esp_err_to_name(ret));
            return false;
        }
        callback_registered_ = true;
    }

    // Now that Bluedroid has accepted a GATTS callback, the stack is up:
    // set the device name here (not in setup(), where it would be too early).
    esp_ble_gap_set_device_name(device_name_.c_str());

    // Register our app id only once. If REG_EVT later reports failure we
    // reset the pending flag so a retry is possible.
    if (!app_register_pending_ && gatts_if_ == ESP_GATT_IF_NONE) {
        esp_err_t ret = esp_ble_gatts_app_register(0x5A);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "gatts_app_register failed (%s), will retry",
                     esp_err_to_name(ret));
            return false;
        }
        app_register_pending_ = true;
    }

    ESP_LOGI(TAG, "GATTS registration started.");
    return true;
}

void BLEMiRemote::create_services_() {
    services_created_ = 0;
    esp_ble_gatts_create_attr_tab(DIS_ATTR_DB, gatts_if_, IDX_DIS_MAX, INST_DIS);
    esp_ble_gatts_create_attr_tab(BAS_ATTR_DB, gatts_if_, IDX_BAS_MAX, INST_BAS);
    esp_ble_gatts_create_attr_tab(HID_ATTR_DB, gatts_if_, IDX_HID_MAX, INST_HID);
}

void BLEMiRemote::configure_adv_() {
    state_ = State::ADV_CONFIGURING;
    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "config_adv_data failed: %s", esp_err_to_name(ret));
    }
}

void BLEMiRemote::start_advertising_() {
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start_advertising failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Advertising started.");
        state_ = State::ADVERTISING;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Report sending
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::send_consumer_report_(uint16_t key) {
    consumer_report_buf[0] = key & 0xFF;
    consumer_report_buf[1] = (key >> 8) & 0xFF;
    if ((consumer_cccd_ & 0x0001) && conn_id_ != 0xFFFF) {
        esp_ble_gatts_send_indicate(gatts_if_, conn_id_,
                                    hid_handles_[IDX_HID_CONSUMER_VAL],
                                    sizeof(consumer_report_buf),
                                    consumer_report_buf, false);
    }
}

void BLEMiRemote::send_keyboard_report_(uint8_t key) {
    memset(keyboard_report_buf, 0, sizeof(keyboard_report_buf));
    keyboard_report_buf[2] = key;  // [0]=modifiers, [1]=reserved, [2..7]=keys
    if ((keyboard_cccd_ & 0x0001) && conn_id_ != 0xFFFF) {
        esp_ble_gatts_send_indicate(gatts_if_, conn_id_,
                                    hid_handles_[IDX_HID_KB_VAL],
                                    sizeof(keyboard_report_buf),
                                    keyboard_report_buf, false);
    }
}

void BLEMiRemote::release_keys_() {
    // Send zeroed consumer report
    uint8_t zero_consumer[2] = {0, 0};
    if ((consumer_cccd_ & 0x0001) && conn_id_ != 0xFFFF) {
        esp_ble_gatts_send_indicate(gatts_if_, conn_id_,
                                    hid_handles_[IDX_HID_CONSUMER_VAL],
                                    sizeof(zero_consumer), zero_consumer, false);
    }
    // Send zeroed keyboard report
    uint8_t zero_kb[8] = {0};
    if ((keyboard_cccd_ & 0x0001) && conn_id_ != 0xFFFF) {
        esp_ble_gatts_send_indicate(gatts_if_, conn_id_,
                                    hid_handles_[IDX_HID_KB_VAL],
                                    sizeof(zero_kb), zero_kb, false);
    }
}

void BLEMiRemote::handle_button_press(uint8_t type) {
    if (type >= BTN_COUNT) return;
    if (state_ != State::CONNECTED) {
        ESP_LOGW(TAG, "Button pressed but not connected.");
        return;
    }

    uint16_t ckey = CONSUMER_KEY[type];
    uint8_t  kkey = KEYBOARD_KEY[type];

    if (ckey != 0) {
        send_consumer_report_(ckey);
    } else if (kkey != 0) {
        send_keyboard_report_(kkey);
    }

    // Schedule key release in 50 ms
    pending_release_ = true;
    release_at_ = millis() + 50;
}

// ─────────────────────────────────────────────────────────────────────────────
// GAP event handler  (called by ESPHome's ESP32BLE dispatcher)
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::gap_event_handler(esp_gap_ble_cb_event_t event,
                                     esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGD(TAG, "ADV data set, starting advertising.");
            start_advertising_();
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "ADV start failed: %d", param->adv_start_cmpl.status);
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGD(TAG, "ADV stopped.");
            break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
            // Accept pairing request from remote device
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(TAG, "Pairing successful.");
            } else {
                ESP_LOGW(TAG, "Pairing failed, reason: 0x%02X",
                         param->ble_security.auth_cmpl.fail_reason);
            }
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GATTS static trampoline
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::gatts_cb_static_(esp_gatts_cb_event_t event,
                                    esp_gatt_if_t gatts_if,
                                    esp_ble_gatts_cb_param_t *param) {
    if (global_instance != nullptr) {
        global_instance->gatts_event_(event, gatts_if, param);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// GATTS event handler
// ─────────────────────────────────────────────────────────────────────────────
void BLEMiRemote::gatts_event_(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        // ── App registered ────────────────────────────────────────────────
        case ESP_GATTS_REG_EVT:
            if (param->reg.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "GATTS reg failed: %d", param->reg.status);
                app_register_pending_ = false;
                state_ = State::IDLE;
                break;
            }
            gatts_if_ = gatts_if;
            app_register_pending_ = false;
            ESP_LOGI(TAG, "GATTS app registered (gatts_if=%d).", gatts_if_);

            // Security parameters: JustWorks bonding, no MITM
            {
                esp_ble_auth_req_t auth = ESP_LE_AUTH_BOND;
                esp_ble_io_cap_t   iocap = ESP_IO_CAP_NONE;
                uint8_t key_size = 16;
                uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
                uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
                esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth,     1);
                esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    1);
                esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, 1);
                esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, 1);
                esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  1);
            }

            state_ = State::CREATING;
            create_services_();
            break;

        // ── Attribute table created ───────────────────────────────────────
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            if (param->add_attr_tab.status != ESP_GATT_OK) {
                ESP_LOGE(TAG, "create_attr_tab failed (inst=%d): %d",
                         param->add_attr_tab.srvc_inst_id, param->add_attr_tab.status);
                break;
            }

            uint8_t inst = param->add_attr_tab.srvc_inst_id;
            uint16_t num = param->add_attr_tab.num_handle;

            if (inst == INST_DIS && num >= IDX_DIS_MAX) {
                memcpy(dis_handles_, param->add_attr_tab.handles,
                       sizeof(uint16_t) * IDX_DIS_MAX);
                esp_ble_gatts_start_service(dis_handles_[IDX_DIS_SVC]);
                services_created_++;
                ESP_LOGD(TAG, "DIS service started.");
            } else if (inst == INST_BAS && num >= IDX_BAS_MAX) {
                memcpy(bas_handles_, param->add_attr_tab.handles,
                       sizeof(uint16_t) * IDX_BAS_MAX);
                esp_ble_gatts_start_service(bas_handles_[IDX_BAS_SVC]);
                services_created_++;
                ESP_LOGD(TAG, "BAS service started.");
            } else if (inst == INST_HID && num >= IDX_HID_MAX) {
                memcpy(hid_handles_, param->add_attr_tab.handles,
                       sizeof(uint16_t) * IDX_HID_MAX);
                esp_ble_gatts_start_service(hid_handles_[IDX_HID_SVC]);
                services_created_++;
                ESP_LOGD(TAG, "HID service started.");
            }

            if (services_created_ == 3) {
                ESP_LOGI(TAG, "All services created, configuring advertising.");
                configure_adv_();
            }
            break;
        }

        // ── Client connected ──────────────────────────────────────────────
        case ESP_GATTS_CONNECT_EVT:
            conn_id_ = param->connect.conn_id;
            memcpy(remote_bda_, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            state_ = State::CONNECTED;
            consumer_cccd_ = 0;
            keyboard_cccd_ = 0;
            battery_cccd_  = 0;
            ESP_LOGI(TAG, "Connected: %02X:%02X:%02X:%02X:%02X:%02X",
                     remote_bda_[0], remote_bda_[1], remote_bda_[2],
                     remote_bda_[3], remote_bda_[4], remote_bda_[5]);
            // Request connection parameter update for lower latency
            {
                esp_ble_conn_update_params_t params = {
                    .min_int = 0x10,
                    .max_int = 0x20,
                    .latency = 0,
                    .timeout = 400,
                };
                memcpy(params.bda, remote_bda_, sizeof(esp_bd_addr_t));
                esp_ble_gap_update_conn_params(&params);
            }
            break;

        // ── Client disconnected ───────────────────────────────────────────
        case ESP_GATTS_DISCONNECT_EVT:
            conn_id_ = 0xFFFF;
            state_   = State::ADVERTISING;
            ESP_LOGI(TAG, "Disconnected (reason: 0x%02X).",
                     param->disconnect.reason);
            if (reconnect_) {
                ESP_LOGI(TAG, "Restarting advertising.");
                start_advertising_();
            }
            break;

        // ── Write to CCCD (enable/disable notifications) ──────────────────
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep && param->write.len == 2) {
                uint16_t val = param->write.value[0] | (param->write.value[1] << 8);
                uint16_t hdl = param->write.handle;

                if (hdl == hid_handles_[IDX_HID_CONSUMER_CCCD]) {
                    consumer_cccd_ = val;
                    ESP_LOGD(TAG, "Consumer CCCD = 0x%04X", val);
                } else if (hdl == hid_handles_[IDX_HID_KB_CCCD]) {
                    keyboard_cccd_ = val;
                    ESP_LOGD(TAG, "Keyboard CCCD = 0x%04X", val);
                } else if (hdl == bas_handles_[IDX_BAS_CHAR_LEVEL_CCCD]) {
                    battery_cccd_ = val;
                    ESP_LOGD(TAG, "Battery CCCD = 0x%04X", val);
                }
            }
            break;

        // ── MTU negotiation ───────────────────────────────────────────────
        case ESP_GATTS_MTU_EVT:
            ESP_LOGD(TAG, "MTU set to %d", param->mtu.mtu);
            break;

        default:
            break;
    }
}

}  // namespace ble_mi_remote
}  // namespace esphome

#endif  // USE_ESP32
