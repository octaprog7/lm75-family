# LM75-family
## [На русском](README_RU.md)
Micropython module for control LM75 like (LM75, LM75A/B, TMP75, TMP175, TMP275) temperature sensors.

## Supported Sensors
| Sensor        | Resolution | Accuracy  | ONE_SHOT |
|---------------|------------|-----------|----------|
| LM75          | 0.5 C      | +/-2.0 C  | No       |
| LM75A/B       | 0.125 C    | +/-0.5 C  | No       |
| TMP75/175/275 | 0.0625 C   | +/-0.25 C | No       |

# Connection
Just connect your LM75 like sensor board to Arduino, ESP or any other board with MicroPython firmware.

Supply voltage 3.3 volts! Use four wires to connect (I2C).
1. +VCC (Supply voltage)
2. GND
3. SDA
4. SCL

Upload micropython firmware to the NANO(ESP, etc.) board, and then files: main_75.py, lm75mod.py and sensor_pack_2 folder. 
Then open main_75.py in your IDE and run it.

# Pictures
## IDE
![alt text](https://github.com/octaprog7/lm75-family/blob/master/pics/lm75AB_ide.png)
![alt text](https://github.com/octaprog7/lm75-family/blob/master/pics/lm75b_temp_meas.png)

# Test results
* [LM75BD](/test_result/lm75bd.txt)
* [TMP75CQ](/test_result/tmp75cq.txt)

# Class hierarchy
```
LM75LikeBase (base class)
├── LM75
│   ├── LM75AB (high accuracy: 0.125 C)
│   └── LM75CD (standard accuracy: 0.5 C)
└── TMP75
    ├── TMP175 (alias)
    └── TMP275 (alias)
```
## Class and sensor mapping (lm75mod.py)
| Class  | Supported sensors     | Resolution (LSB) | Accuracy (typ.) | Range        | ONE_SHOT |
|--------|-----------------------|------------------|-----------------|--------------|----------|
| LM75   | LM75, LM75C, LM75D    | 0.5 C            | +/-2.0 C        | -55...+125 C | No       |
| LM75AB | LM75A, LM75B          | 0.125 C          | +/-0.5 C        | -55...+125 C | No       |
| LM75CD | LM75, LM75C, LM75D    | 0.5 C            | +/-2.0 C        | -55...+125 C | No       |
| TMP75  | TMP75, TMP175, TMP275 | 0.0625 C         | +/-0.25 C       | -40...+125 C | No       |
| TMP175 | TMP175 (alias TMP75)  | 0.0625 C         | +/-0.25 C       | -40...+125 C | No       |
| TMP275 | TMP275 (alias TMP75)  | 0.0625 C         | +/-0.25 C       | -40...+125 C | No       |

# License
MIT license

## Notes
Note: This driver is tested on real hardware with LM75BD, TMP75CQ.

## Quick Start
```python
from machine import I2C, Pin
from sensor_pack_2.bus_service import I2cAdapter
from lm75mod import LM75AB

# Initialize
i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
adapter = I2cAdapter(i2c)
ts = LM75AB(adapter=adapter, address=0x48)

# Read temperature
ts.start_measurement()
print(f"Temperature: {ts.get_measurement_value():.3f} C")
```

# Troubleshooting
| Problem                  | Possible Cause          | Solution                                 |
|--------------------------|-------------------------|------------------------------------------|
| ALERT=True always        | Therm Mode + hysteresis | Cool below Tmin or switch to Alert Mode  |
| No I2C communication     | Wrong address/pull-ups  | Check A0-A2, 4.7kΩ pull-up on SDA/SCL    |
| Inaccurate readings      | Self-heating/mounting   | Use set_temperature_offset()             |
| ValueError on thresholds | Window too narrow       | Increase T_max - T_min (min 3× accuracy) |

# Support the project
If you found this driver helpful, please rate it!
This helps us develop the project and add support for new sensors.
If you liked my software, please be generous and give it a star!

# Author
Roman Shevchik <goctaprog@gmail.com>