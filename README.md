# smart-ring

A working smart ring on a custom flexible PCB. Optical heart rate, HRV, SpO2, respiration
rate and skin temperature, computed on the ring and streamed over Bluetooth Low Energy.
nRF52832 with a TI AFE4404 optical front end, an ICM-42670 IMU used as a motion reference
for adaptive artifact cancellation, and a TMP117 temperature sensor. Logs to internal flash
when no phone is connected and replays the backlog on reconnect.

<!-- PHOTO: all board revisions laid out in order, earliest to latest -->

Electronics complete and validated across multiple board revisions. Enclosure not yet fabricated.

## What it measures

| Signal | How |
|---|---|
| Heart rate | Green channel PPG, adaptive threshold beat detection, EWMA smoothed. 40 to 180 BPM range. |
| HRV | SDNN and RMSSD over a 32 beat RR interval buffer, minimum 12 beats before reporting. |
| SpO2 | Red / IR ratio of ratios, classic linear fit. Uncalibrated against a reference oximeter, so treat as indicative. |
| Respiration rate | Zero crossings of the PPG baseline wander, gated to 0.8 to 8 s per breath. |
| Stress index | Simple linear map from RMSSD, 0 to 100. A heuristic, not a validated model. |
| Skin temperature | TMP117, 0.0078 C resolution. |
| Motion | ICM-42670 accel and gyro at 100 Hz, feeds the artifact filter and a still / moving / heavy gate. |

## Hardware

| Part | Role |
|---|---|
| Nordic nRF52832 | Arm Cortex M4F, 64 MHz, 512 KB flash, 64 KB RAM. BLE 5 radio. |
| TI AFE4404 | Three channel optical AFE driving green, red and IR LEDs with hardware ambient subtraction. 100 Hz sample rate, ADC_RDY interrupt as the master clock. |
| TDK ICM-42670 | 6 axis IMU, accel at plus or minus 16 g and gyro at 2000 dps, 100 Hz, matched to the AFE. |
| TI TMP117 | High accuracy digital temperature sensor. |
| Custom flexible PCB | Single I2C bus. Several revisions to get optical placement and signal quality right. |

An earlier revision used a Bosch BMI088 IMU. Its bring up sketch is kept in `firmware/bringup/`.

<!-- PHOTO: close up of the latest board -->

### Why flex

A ring has no flat surface. A rigid board would need to be tiny and stacked, which kills the
sensor to skin contact the PPG needs. Flex lets the board follow the inside of the band so the
optical window sits flat against the finger.

<!-- Add: layer count, material, design tool, fab house, dimensions, and the revision history.
     Hardware people read the revision history first. -->

## Firmware

Built on the Adafruit nRF52 Arduino core with the S132 SoftDevice for the BLE stack, chosen
for iteration speed. The sensor drivers are register level, not library calls: the AFE4404
timing engine, gain and LED current registers are configured directly, as are the ICM-42670
and TMP117.

### Signal chain, per 100 Hz sample

1. Read the green channel from the AFE on the ADC_RDY interrupt.
2. Median filter over 7 samples to kill single sample spikes.
3. Track DC with a leaky integrator and subtract it to get the AC pulse component.
4. Read the IMU, high pass the accelerometer to remove gravity, and run a 4 tap NLMS adaptive
   filter using the three accelerometer axes as noise references. The filter output is the
   PPG with the motion correlated component removed.
5. Adaptive threshold peak detection on the cleaned signal, with a blanking window derived
   from the last beat interval and a threshold that decays when no beat is seen.
6. Reject beats when motion exceeds a hard limit or a peak is implausibly large relative to
   the running average.
7. Compute BPM from the last 8 valid RR intervals and smooth with an EWMA that weights
   history more heavily while moving.

The NLMS step is what makes heart rate hold up while the hand is moving. Before it, a wrist
flick registered as several beats.

### Bring up and auto tune

On boot the firmware runs an I2C bus recovery sequence (nine clock pulses to free a stuck
slave), resets the AFE, and sweeps TIA gain and LED current until the green channel DC level
lands in a target ADC window. This means the same firmware works across board revisions with
different optical geometry without hand tuning.

### BLE

Nordic UART Service. The AFE samples at 100 Hz but small notifies at that rate are unreliable,
so packets are decimated 4 to 1 and sent at about 25 Hz as newline delimited CSV with 17 fields:

    ir, red, amb, bpm_x10, spo2, sdnn, rmssd, stress, resp_x10, beat,
    temp_x100, ax_x1000, ay_x1000, az_x1000, gx_x10, gy_x10, gz_x10

Live packets are prefixed `L,`. History packets replayed from flash are prefixed `H,`.

### Offline logging

When no client is connected the ring keeps sampling and writes one 20 byte summary record to
internal flash every 30 seconds via LittleFS: epoch, BPM, SpO2, temperature, SDNN, RMSSD,
respiration, beat count, activity and flags. Capacity is 1500 records, about 12.5 hours. On
reconnect the backlog is dumped, an `END` marker is sent, and the file is erased.

### Debug

SEGGER RTT over J-Link. Every sketch prints its state on boot and the lean HR sketch prints
one line per detected beat with the motion classification.

### Power

<!-- Add a real number here. Measure average current with a multimeter or a Nordic PPK2
     at idle, sensing only, and sensing plus BLE connected. One measured figure is worth
     more than any prose. Until then, say only what is true: LED currents are kept low
     (green code 20, IR code 3) and BLE traffic is decimated 4 to 1. -->

## Client

Three browser dashboards in `client/` using Web Bluetooth. No app install needed on desktop
Chrome or Edge.

- `pulse_monitor.html`: live PPG waveform, BPM, SpO2, HRV, respiration
- `full_monitor.html`: everything above plus temperature and IMU
- `imu_viewer.html`: accelerometer and gyro traces

Open the file, click connect, pick the ring.

## Repo layout

    firmware/ring_monitor/      Full firmware: PPG, IMU, temp, BLE, flash logging. Flash this one.
    firmware/hr_motion_cancel/  Lean sketch isolating the NLMS motion cancellation work.
    firmware/history/           The progression, numbered. RTT only, then BLE, then three
                                channel, then full sensor set. Kept for reference.
    firmware/bringup/           Per sensor test sketches and an I2C scanner.
    firmware/findmy_beacon/     Side experiment: the ring advertising as an Apple Find My
                                beacon. Key removed, bring your own.
    client/                     Web Bluetooth dashboards.
    docs/                       Photos and measurements.

Sketches in `history/` and `bringup/` are named descriptively rather than to match their
folder, so to build one in the Arduino IDE copy it into a folder of the same name first.

## Build

Arduino IDE 2.x with the Adafruit nRF52 board package. Board: Adafruit Feather nRF52832
(the custom board uses the same MCU and pinout mapping). Flash the S132 SoftDevice 6.1.1
once via the bootloader, then upload `ring_monitor`. Debug output via J-Link RTT Viewer.

## Status

Working on the bench. Next: enclosure, a measured power figure, then a multi hour wear test
against a reference chest strap.

## License

MIT.
