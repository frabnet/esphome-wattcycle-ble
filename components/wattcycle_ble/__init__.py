import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor", "text_sensor"]
MULTI_CONF = True

wattcycle_ble_ns = cg.esphome_ns.namespace("wattcycle_ble")
WattCycleBLE = wattcycle_ble_ns.class_(
    "WattCycleBLE",
    cg.Component,
    ble_client.BLEClientNode,
)

CONF_WATTCYCLE_BLE_ID = "wattcycle_ble_id"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_POLL_OFFSET = "poll_offset"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_BATTERY_CURRENT = "battery_current"
CONF_BATTERY_POWER = "battery_power"
CONF_MOS_TEMPERATURE = "mos_temperature"
CONF_PCB_TEMPERATURE = "pcb_temperature"
CONF_SOC = "soc"
CONF_REMAINING_CAPACITY = "remaining_ah"
CONF_TOTAL_CAPACITY = "total_ah"
CONF_DESIGN_CAPACITY = "design_ah"
CONF_CYCLES = "cycles"
CONF_CELL_VOLTAGES = "cells_v"
CONF_CELL_SPREAD = "cell_spread_mv"
CONF_CELL_TEMPERATURES = "cell_temps"
CONF_MODEL_OR_FW = "model_or_fw"
CONF_MANUFACTURER = "manufacturer"
CONF_SERIAL = "serial"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WattCycleBLE),
        cv.Required(ble_client.CONF_BLE_CLIENT_ID): cv.use_id(ble_client.BLEClient),
        cv.Optional(CONF_UPDATE_INTERVAL, default="10s"): cv.update_interval,
        cv.Optional(CONF_POLL_OFFSET, default="0s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MOS_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PCB_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_SOC): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REMAINING_CAPACITY): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TOTAL_CAPACITY): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DESIGN_CAPACITY): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CYCLES): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_SPREAD): sensor.sensor_schema(
            unit_of_measurement="mV",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGES): cv.ensure_list(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        cv.Optional(CONF_CELL_TEMPERATURES): cv.ensure_list(
            sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            )
        ),
        cv.Optional(CONF_MODEL_OR_FW): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_MANUFACTURER): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SERIAL): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[ble_client.CONF_BLE_CLIENT_ID])
    cg.add(var.set_parent(parent))

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_poll_offset(config[CONF_POLL_OFFSET]))

    if CONF_BATTERY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_VOLTAGE])
        cg.add(var.set_battery_voltage_sensor(sens))

    if CONF_BATTERY_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_CURRENT])
        cg.add(var.set_battery_current_sensor(sens))

    if CONF_BATTERY_POWER in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_POWER])
        cg.add(var.set_battery_power_sensor(sens))

    if CONF_MOS_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_MOS_TEMPERATURE])
        cg.add(var.set_mos_temperature_sensor(sens))

    if CONF_PCB_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_PCB_TEMPERATURE])
        cg.add(var.set_pcb_temperature_sensor(sens))

    if CONF_SOC in config:
        sens = await sensor.new_sensor(config[CONF_SOC])
        cg.add(var.set_soc_sensor(sens))

    if CONF_REMAINING_CAPACITY in config:
        sens = await sensor.new_sensor(config[CONF_REMAINING_CAPACITY])
        cg.add(var.set_remaining_capacity_sensor(sens))

    if CONF_TOTAL_CAPACITY in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_CAPACITY])
        cg.add(var.set_total_capacity_sensor(sens))

    if CONF_DESIGN_CAPACITY in config:
        sens = await sensor.new_sensor(config[CONF_DESIGN_CAPACITY])
        cg.add(var.set_design_capacity_sensor(sens))

    if CONF_CYCLES in config:
        sens = await sensor.new_sensor(config[CONF_CYCLES])
        cg.add(var.set_cycles_sensor(sens))

    if CONF_CELL_SPREAD in config:
        sens = await sensor.new_sensor(config[CONF_CELL_SPREAD])
        cg.add(var.set_cell_spread_sensor(sens))

    for conf in config.get(CONF_CELL_VOLTAGES, []):
        sens = await sensor.new_sensor(conf)
        cg.add(var.add_cell_voltage_sensor(sens))

    for conf in config.get(CONF_CELL_TEMPERATURES, []):
        sens = await sensor.new_sensor(conf)
        cg.add(var.add_cell_temperature_sensor(sens))

    if CONF_MODEL_OR_FW in config:
        sens = await text_sensor.new_text_sensor(config[CONF_MODEL_OR_FW])
        cg.add(var.set_model_or_fw_text_sensor(sens))

    if CONF_MANUFACTURER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_MANUFACTURER])
        cg.add(var.set_manufacturer_text_sensor(sens))

    if CONF_SERIAL in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SERIAL])
        cg.add(var.set_serial_text_sensor(sens))
