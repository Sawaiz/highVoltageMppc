# MPPC High Voltage
Surface mount module that provides 52.5V to 57.5V in 20mV increments. Also contains a 2 input 24bit ADC to allow for closed loop control of the voltage, and read in a thermistor mounted near the sensor.

![][withEMIshield]

## Hardware
The module uses a MAX1932 APD bias voltage boost converter and NAU7802SGI 24bit Analog to Digital converter (ADC) for reading temperature and output voltage.

![][noShieldFront]

### High Voltage
The module uses a [MAX1932](http://datasheets.maximintegrated.com/en/ds/MAX1932.pdf) APD bias voltage boost converter to maintain the output voltage. The voltage range, and step size is dictated with setting R2, R3, and R4. The minimum and maximum voltage set using the equations for on page 7 of the datasheet, and there are 8 bits (256) intermediate values that are possible. The high switching portion was designed with general SMPS design requirements (short high current loops, separated ground planes, EMI shielding) providing a low ripple output (<5mVpp ripple) and no significant back EMF.

### ADC
The analog to digital converter is a one designed for high precision and low sampling rate. The [NAU7802SGI](http://www.nuvoton.com/resource-files/NAU7802%20Data%20Sheet%20V1.7.pdf) is marketed at the load sensing market, but the high precision and low sampling rate lends itself well to the closed loop control and monitoring application. The high voltage is divided by 100 and the thermistor is put across a equivalent 100K static resistor to go along with he 100K NTC found on the MPPC sensor module to act as a voltage divider.

## Software
Each module has two buses that run to it. The following lists pins and their functions.

| Pin | Label | Description               |
| --- | ----- | ------------------------- |
|1    | 3V3   | Regulated 3.3V in         |
|2    | CS    | **SPI** bus Chip Select   |
|3    | SCLK  | **SPI** bus clock         |
|4    | DIN   | **SPI** bus data in       |
|5    | GND   | Common Ground             |
|6    | DRDY  | ADC conversion done       |
|7    | SCL   | **I2C** bus clock         |
|8    | SDA   | **I2C** bus data          |

### High Voltage
The SPI bus is connected directly to the MAX1932. Chip select is active low and data is sent MSB (most significant bit first). Sending bytes to it sets the voltage. Setting them to 0x00 turns off the DC-DC converter portion. As values increase to 0xFF (127), the output voltage falls, linearly.

```Arduino
//Example Arduino code that loops through all voltages.
//SCLK --> Pin 13 for Uno or Duemilanove
//DIN ---> Pin 11 for Uno or Duemilanove

#include <SPI.h>
#define CS 2 //Any pin

void setup(){
	Serial.begin(9600);
	pinMode(CS, OUTPUT);
	digitalWrite(CS, HIGH);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
}

void loop(){
	for(int i = 0 ; i < 256 ; i++){
		digitalWrite(CS, LOW);
		SPI.transfer(i);
		digitalWrite(CS, HIGH);
		Serial.println(i);
	}
}
```

### ADC
The ADC communicates through the I2C bus, the device address is permanently programed to *0101010* (0x2A). It is fully compliant with the I2C protocol and the datasheet has the register map (page 28) and protocol description (page 14).

```Arduino
//Example Arduino code to read back the two inputs.
#include <Wire.h>
unsigned long adcVal;

void setup(){
	Serial.begin(9600);
	Wire.begin();
}

void loop(){
	// step 1: instruct sensor to read
	Wire.beginTransmission(42); // transmit to device #42 (0x2A)
	Wire.write(byte(0x00));      // control address byte (0x)
	Wire.write(byte(0x00));      // data byte (0x)
	Wire.endTransmission();      // stop transmitting

	// step 2: wait for readings to happen
	delay(1000);

	// step 3: request reading from sensor
	Wire.requestFrom(42, 3);    // request 2 bytes from slave device #42

	// step 4: receive reading from sensor
	if (3 <= Wire.available()) { // if two bytes were received
		adcVal = Wire.read();      // receive byte one (overwrites previous reading)
		adcVal = adcVal << 8;      // shift byte to make room for new byte
		adcVal = Wire.read();      // receive high two
		adcVal = adcVal << 8;      // shift both bytes
		adcVal |= Wire.read();     // receive low byte
		Serial.println(adcVal);    // print the reading
	}
}
```

## Archived
Archived, HV intergrated into MPPC interface from v1.1.

![][mppcBack]

[mppcBack]: cad/renderings/mppcBack.jpg "View of Back of Board"
[noShieldFront]: cad/renderings/noShieldFront.jpg "Direct front view"
[withEMIshield]: cad/renderings/withEMIshield.jpg "Board rendering with EMI shield"
