# spcd-kernel-module
Kernel Module for the SPCD Device, which exposes control point interfaces, but does **not** include control algorithms.

## Character Device Interface
The driver implements a character buffer interface at `/dev/spcd0` that supports select() polling for interrupt based inputs and a device command set.

### Inputs
Reads occur one byte at a time. Each byte is a packed bitmask.
The high bit, (bit 7) is set at power-on as a one-shot read of the dealer_enable during driver initialization. 
If it's high, then the software should start into 'dealer' mode.
```
     X  preboot_stat
     |  |  12v
     |  |  |  failsafe
     |  |  |  |  valve_open
     |  |  |  |  |  overpressure
     |  |  |  |  |  |  stuck_on
     |  |  |  |  |  |  |  dealer_enable
     |  |  |  |  |  |  |  |
Bit: 7  6  5  4  3  2  1  0
```

### Control Commands
Writes to the character device are commands. Each command has an expected structure:

#### SET_BLOWER_PWM
One byte command followed by two u64 values representing nanoseconds of period and duty of the PWM.
```
| 0x01 | u64 (period) | u64 (duty cycle) |
```
#### SET_VALVE_PWM
One byte command followed by a list of valve PWM states to iterate in a loop.
Each valve state has a period, duty-cycle, and duration to hold the state before progressing to the next state.
```
Command: 
| 0x02 | u8 (number of cycles) | cycle_definition |

cycle_definition:
| u64 (period) | u64 (duty cycle) | u64 (duration) |
```

Example: Alternate the valve between two duty cycles every 5000000 ns:
You'd write a u8, followed by 
```
    u8 Command
    |     u8 Cycle Count
    |     |               (6) u64 defining the two cycles.
    |     |               |
| 0x02 | 0x02 | 0x0000000001312D00 | 0x0000000002625A00 | 0x00000000004C4B40 | 0x000000000065B9AA | 0x0000000002625A00 | 0x00000000004C4B40 |
```

#### START_VALVE_CYCLE
Single-byte command, `0x04`.

#### STOP_VALVE_CYCLE
Single-byte command, `0x08`.


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
preboot_stat
```
Example: To read the immediate value of `mode_switch`.

`cat /sys/bus/platform/devices/spcd@0/mode_switch`

### Outputs:
The following files are read/write and will respond to `0` or `1` being written to the file:
```
pwr_hold
postboot_stat
one_min
```

#### PWM:
The following outputs control the hardware PWM timers for the blower and valve.
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

To disable PWM, set the duty cycles to zero.