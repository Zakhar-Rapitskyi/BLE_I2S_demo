# Zakhar Rapitksyi Demo with BLE + I2S

## Features

- **BLE Custom Service** with two characteristics:
  - Audio Control (write-only, 1 byte) - Play/stop melodies
  - Accelerometer Data (read-only, 6 bytes) - X, Y, Z axis readings
- **I2S Audio Playback** - Two pre-loaded melodies in flash
- **Motion Detection** - BMA280 accelerometer with interrupt-driven updates

### Components
- **nRF52832 Development Kit (PCA10040)**
- **BMA280 Accelerometer Module** (I2C interface)
- **I2S Audio Output** (speaker/amplifier)

### Pin Connections

#### I2C (Accelerometer)
| nRF52832 Pin | BMA280 Pin |
|--------------|------------|
| P0.26 (SDA)  | SDA        |
| P0.27 (SCL)  | SCL        |
| GND          | GND        |
| VDD          | VCC        |

#### I2S (Audio module)
| nRF52832 Pin | Audio Device |
|--------------|--------------|
| P0.28        | SCK          |
| P0.29        | LRCK         |
| P0.30        | SDOUT        |
| P0.31        | MCK          |

---

## **KNOWN HARDWARE ISSUE: BMA280 INT Pin Not Connected**

### Problem Description

The BMA280 accelerometer module provided for this project **does not have the INT1 pin physically connected** to any accessible pin on the module PCB. This creates a hardware limitation that prevents the use of motion detection interrupts as originally specified in the requirements.

### Current Workaround Solution

**Debug Mode with Button Simulation:**

The firmware includes a debug mode (`USE_ACCEL_WORKAROUND`) that simulates accelerometer interrupts using a button press:

```c
#define USE_ACCEL_WORKAROUND 1 /**< TODO: SET TO 0 WHEN NORMAL ACCELEROMETER WILL BE PRESENT */
```

**How it works:**
1. Button 1 on nRF52 DK is configured as input
2. When pressed, it drives GPIO P0.02 HIGH
3. GPIO P0.02 is connected to P0.25 (configured as accelerometer INT pin)
4. This simulates the BMA280 INT1 signal


### Alternative Solutions

#### Option 1: Polling Mode (Software Workaround)
Read accelerometer data periodically via I2C instead of waiting for interrupts:

**Pros:**
- Works with current hardware
- No external wiring needed

**Cons:**
- Higher power consumption (periodic I2C reads)
- Increased CPU usage
- Slower response time
- Cannot wake from deep sleep on motion

#### Option 2: Hardware Rework (Recommended for Production)
**Modify the BMA280 module to expose INT1 pin:**

1. Locate INT1 pad on BMA280 IC (consult datasheet)
2. Solder a thin wire to INT1 pad
3. Connect to nRF52 P0.25
4. Set `USE_ACCEL_WORKAROUND  0` in firmware

**Pros:**
- Lowest power consumption
- Meets original specification
- Instant motion detection

**Cons:**
- Risk of damaging module

#### Option 3: Different Accelerometer Module
Use a module with INT pin properly exposed
