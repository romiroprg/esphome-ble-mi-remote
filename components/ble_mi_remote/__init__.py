import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32_ble"]
AUTO_LOAD = ["button"]

ble_mi_remote_ns = cg.esphome_ns.namespace("ble_mi_remote")
BLEMiRemote = ble_mi_remote_ns.class_("BLEMiRemote", cg.Component)
MiRemoteButton = ble_mi_remote_ns.class_("MiRemoteButton", button.Button)

CONF_MANUFACTURER_ID = "manufacturer_id"
CONF_BATTERY_LEVEL = "battery_level"
CONF_RECONNECT = "reconnect"

# btn_name → (setter, type_index)
BUTTONS = {
    "power":        ("set_power_button",        0),
    "home":         ("set_home_button",         1),
    "back":         ("set_back_button",         2),
    "menu":         ("set_menu_button",         3),
    "up":           ("set_up_button",           4),
    "down":         ("set_down_button",         5),
    "left":         ("set_left_button",         6),
    "right":        ("set_right_button",        7),
    "ok":           ("set_ok_button",           8),
    "volume_up":    ("set_volume_up_button",    9),
    "volume_down":  ("set_volume_down_button",  10),
    "mute":         ("set_mute_button",         11),
    "play_pause":   ("set_play_pause_button",   12),
    "fast_forward": ("set_fast_forward_button", 13),
    "rewind":       ("set_rewind_button",       14),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BLEMiRemote),
        cv.Optional("name", default="MI Remote"): cv.string,
        cv.Optional(CONF_MANUFACTURER_ID, default="Xiaomi"): cv.string,
        cv.Optional(CONF_BATTERY_LEVEL, default=100): cv.int_range(min=0, max=100),
        cv.Optional(CONF_RECONNECT, default=True): cv.boolean,
        **{cv.Optional(k): button.button_schema(MiRemoteButton) for k in BUTTONS},
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_device_name(config["name"]))
    cg.add(var.set_manufacturer_name(config[CONF_MANUFACTURER_ID]))
    cg.add(var.set_battery_level(config[CONF_BATTERY_LEVEL]))
    cg.add(var.set_reconnect(config[CONF_RECONNECT]))

    for btn_name, (setter, _type_idx) in BUTTONS.items():
        if btn_name in config:
            btn = await button.new_button(config[btn_name])
            cg.add(getattr(var, setter)(btn))
