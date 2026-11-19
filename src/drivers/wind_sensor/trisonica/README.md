# LI-550 TriSonica anemometer on Pixhawk 6C

Driver for the LI-COR LI-550 TriSonica Mini ultrasonic anemometer. It publishes
horizontal wind speed and direction on the uORB topic `sensor_airflow`, which is
bridged to ROS 2 as `/fmu/out/sensor_airflow`.

## Build and flash

The branch is `feature/trisonica-wind-sensor` and is based on PX4 v1.16.2.

```sh
make px4_fmu-v6c_default
make px4_fmu-v6c_default upload       # or QGC: Firmware > Advanced > Custom firmware file
```

After flashing, run `param reset_all` and then `reboot` in the MAVLink console.
The board defaults in `boards/px4/fmu-v6c/init/rc.board_defaults` set up the ports:

| Port   | Device       | Use                          | Params                                     |
|--------|--------------|------------------------------|--------------------------------------------|
| TELEM1 | `/dev/ttyS5` | MAVLink (GCS / telemetry)    | `MAV_0_CONFIG=101` (PX4 default)           |
| TELEM2 | `/dev/ttyS3` | uXRCE-DDS (ROS 2), 921600    | `UXRCE_DDS_CFG=102`, `SER_TEL2_BAUD=921600` |
| TELEM3 | `/dev/ttyS1` | TriSonica, 115200 8N1        | `SENS_TRSNC_CFG=103`                       |

The driver always runs at 115200, so no `SER_TEL3_BAUD` parameter appears.
Other parameters:
- `SENS_TRSNC_SETUP`: default 1. Applies the streaming profile on every connection.
- `SENS_TRSNC_RATE`: default 10 Hz.

Neither is saved to the instrument's flash.

## Wiring (LI-550P 12-pin, 3.3 V LVTTL UART)

| Pixhawk 6C TELEM3 | LI-550P                          |
|-------------------|----------------------------------|
| TX (out)          | pin 8, gray: **URX** (UART in)   |
| RX (in)           | pin 11, light green: **UTX** (UART out) |
| GND               | pin 9, white (or 2 / 12)         |

Power the sensor on pin 1 (brown), 5–32 V. Do not use the RS-232 pins
(4 yellow TX, 6 blue RX) with the Pixhawk's 3.3 V UART.

## Checks (MAVLink console)

```
trisonica status           # state should be "streaming"; "listen-only" means the sensor ignores FC TX
listener sensor_airflow
```

If you see `no CLI response, listen-only`, the sensor's data is arriving but the
FC TX → sensor URX path is not working. The driver still publishes, but it parses
the instrument's current output with standard tags and cannot apply
`SENS_TRSNC_SETUP` or `SENS_TRSNC_RATE`.

## ROS 2

```sh
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
ros2 topic echo /fmu/out/sensor_airflow
```

Use a `px4_msgs` package whose `SensorAirflow.msg` matches this branch. PX4
publishes these topics best effort, and `ros2 topic hz` (Jazzy) shows nothing
for them, while `ros2 topic echo` works.
