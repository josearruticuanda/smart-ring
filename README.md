# smart-ring

A working smart ring on a custom flexible PCB. PPG sensing with on device
heart rate, HRV, resting BPM and stress estimation, running from an
integrated rechargeable cell in a ring sized form factor. Firmware on the
nRF52832, telemetry over Bluetooth Low Energy.

[Photo: the board revisions laid out in order, earliest to latest]

Electronics complete and validated across [N] board revisions. Enclosure
not yet fabricated.

## What it measures

Heart rate, heart rate variability ([RMSSD / SDNN / which metric]),
resting BPM, and a stress estimate derived from [what the stress estimate
is based on]. All computed on the ring; the phone only receives results.

## Hardware

Sensor: [PPG part number, e.g. MAX30101] with [green / IR / red] LEDs
MCU: Nordic nRF52832, Arm Cortex M4F at 64 MHz, 512 KB flash, 64 KB RAM
Battery: [capacity] mAh [chemistry], charged over [method]
PCB: [N] layer flexible PCB, [polyimide / material], designed in
[Fusion 360 / KiCad / Altium], fabricated by [fab house]
Board size: [dimensions]

[Photo: close up of the latest board]

### Why flex

A ring has no flat surface. A rigid board would need to be tiny and
stacked, which kills the sensor to skin contact the PPG needs. Flex lets
the board wrap the inside of the band so the optical window sits flat
against the finger. [Add anything specific: how you handled bend radius,
stiffeners under components, connector choice.]

### Board revisions

Rev 1: [what it was, what failed]
Rev 2: [what changed, what you learned]
...
Rev [N]: [current, what works]

[This section matters more than any other. Hardware people read it first.]

## Firmware

Prototyped on Arduino to validate the sensor and signal chain, then
rewritten for the nRF52832 on [nRF5 SDK / Zephyr / nRF Connect SDK] for
production power behaviour.

### Power budget

Continuous optical sensing on a [capacity] mAh cell leaves no margin.
Target runtime: [X hours / days].

What I tuned:
Sampling schedule: PPG at [rate] Hz, [continuous / windowed], LED current
at [mA]
Radio: BLE connection interval [ms], advertising interval [ms], notify
only on [event / interval]
Sleep: System ON idle at [µA], sensor powered down between windows
Achieved: [average current] µA / mA, [runtime] on a full charge

[If you have a current measurement trace or a table, put it here.]

### Signal processing

Raw PPG to heart rate: [bandpass filter range, peak detection method,
window length]
HRV: [RR interval extraction method, artefact rejection]
Stress: [what it's computed from]

[Photo or plot: a raw PPG trace and the detected peaks]

### BLE

Custom GATT service with characteristics for [HR, HRV, battery, raw
stream if you have one]. Payload [bytes] per notification. Designed so
the phone can stay in low power scan and still get a reading every [N]
seconds.

## Client

[One line about the app or tool that receives the data, and a link if
it's public.]

## Repo layout

    firmware/     nRF52832 application
    hardware/     Schematics and board renders. Design files on request.
    prototype/    Arduino validation code
    docs/         Photos, plots, measurements

## Status

Working on the bench. Next: enclosure, then a longer wear test.
