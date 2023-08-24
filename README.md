# spcd-kernel-module
Kernel Module for the SPCD Device, which exposes control point interfaces, but does **not** include control algorithms.

## Sysfs Interface
The sysfs interface populates a virtual filesystem under sysfs. Change the working directory to:
`cd /sys/bus/platform/devices/spcd@0` (which is a link to /sys/devices/soc0/spcd@0) to see options you have for interacting with the spcd.

### Inputs:
The following files are read-only inputs which will contain `0` or `1`:
```
status_12v
stuck_on
valve_open
overpressure
mode_switch
```
Example: To read the immediate value of `mode_switch`.

`cat /sys/bus/platform/devices/spcd@0/mode_switch`

### Outputs:
The following files are read/write and will respond to `0` or `1` being written to the file:
```
pwr_hold
wdt_alert
one_min
```

Additionally, there is a 'buzzer' file that works with `0`, `1`, `2`, `3`, where 0 is off and 3 is high.
Example: To set the buzzer to medium and turn it off again:
```
echo "2" > /sys/bus/platform/devices/spcd@0/buzzer
echo "0" > /sys/bus/platform/devices/spcd@0/buzzer
```

#### PWM:
The following outputs control software PWM using high resolution kernel timers.
```
blower_period
blower_duty_cycle

valve_period
valve_duty_cycle
```

These files are read/write, and specify the timing in _nanoseconds_ for the PWM Square wave.

Example: Set blower 50% duty_cycle cycle at 100Hz
```
echo "10000000" > /sys/bus/platform/devices/spcd@0/blower_period
echo "5000000" > /sys/bus/platform/devices/spcd@0/blower_duty_cycle
```

Example: Read current settings of blower:
```
$ cat /sys/bus/platform/devices/spcd@0/blower_period
8675309
$ cat /sys/bus/platform/devices/spcd@0/blower_duty_cycle
1337
```

To disable PWM, set one (or both) of the values to zero.


