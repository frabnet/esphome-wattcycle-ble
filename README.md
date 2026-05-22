# esphome-wattcycle-ble
WattCycle BLE BMS implementation for ESPHome.\
Tested with: ESPHome 2026.4.3, ESP32 WT32-ETH01,  WattCycle 12V 100Ah Deep Cycle Trolling Motor Battery.\
Should work with all WattCycle Bluetooth series batteries.

# Features
- Supports multiple batteries
- `battery_voltage` Total battery voltage, in V
- `battery_current` Battery current, in A
- `battery_power` Calculated power, in W
- `soc` State of charge, in %
- `remaining_ah` Remaining capacity, in Ah
- `total_ah` Total/full charge capacity, in Ah
- `design_ah` Nominal design capacity, in Ah
- `cycles` Number of battery cycles
- `mos_temperature` MOS/BMS temperature
- `pcb_temperature` PCB/BMS temperature
- `cell_spread_mv` Max-min difference between cells, in mV
- `cells_v` List of individual cell voltages, in V
- `cell_temps` List of cell/probe temperatures, in °C
- `model_or_fw` Model or firmware reported by the BMS
- `manufacturer` Manufacturer field, if provided by the BMS
- `serial` Serial or identification field of the BMS

# How to use

## 1. Enable external component

```yaml
external_components:
  - source: github://frabnet/esphome-wattcycle-ble
```

## 2. Setup BLE connection

```yaml
# BLE tracker
esp32_ble_tracker:

# BLE client
ble_client:
  - mac_address: "C0:D6:XX:XX:XX:XX"
    id: wattcycle_ble_client_batt1

# Optional: switch to enable/disable connection
switch:
  - platform: ble_client
    ble_client_id: wattcycle_ble_client_batt1
    name: "BATT1 BLE Connection"
    restore_mode: RESTORE_DEFAULT_ON
```

## 3. Expose desired entities
```yaml
wattcycle_ble:
  - ble_client_id: wattcycle_ble_client_batt1
    update_interval: 10s
    battery_voltage:
      name: "BATT1 Voltage"
    battery_current:
      name: "BATT1 Current"
    battery_power:
      name: "BATT1 Power"
```
Please refer to `demo.yaml` to see all possible configurations.
