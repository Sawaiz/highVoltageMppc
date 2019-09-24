# High voltage Controller

## Communication
The device acts as a TWI (Two Wire Interface) slave (completely compatible with I2C). It contains X registers which are given the mapping before. Each registers is 8 bits wide, and most are read only, 

### Set Address
The address is set via a trimmer potentiometer. It is connected to a 10-Bit ADC on the device and once sampled 

## Register Map

|-Address |-Contents | R/W |
|---------|------|-----|
| 0x00     |------|-----|
