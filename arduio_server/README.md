# Server software

The server sketch is compiled and loaded into the Arduino using the Arduino IDE. It uses conditional compilation for configuration options such as
debugging verbosity, LED blinking, and serial or WiFi operation. 

Select one only of these definitions
```
#define WIFI
#define SERIAL
```

If there is a Nano Motor carrier shield add this definition

```
#define MOTOR_CARRIER
```

The server can use the onboard LED to indicate its state.  A slow blink means it is alive and processing commands, fast blinking means it is in `run` mode.

```
#define LED
```

To send debugging information on the serial port

```
#define DEBUG
```

To use WiFi communications you must first create the file `arduino_secrets.h` in this folder.  It must contains the following lines:

```
char ssid[] = "";     // your network SSID (name)
char pass[] = "";  // your network password (use for WPA, or use as key for WEP)
```
where you must fill in your network SSID and password.  `.gitignore` is set to prevent this file from being committed to git.

In practice WiFi communications has been found to have quite large and unpredictable latencies.  
Serial communication can easily sustain 50Hz control from an iMac connected via USB serial.

## Input/Output tables

The input and output tables are a linked list of dynamically allocated structs with a head and tail pointer.  Simple list management functions are included.

The devices supported by the server are included in a table, along with a pointer to their "driver" function.  The `#` character is wildcard match for an integer channel or pin number.

```
NF nano[] = {
  { "/adc/#", nano_adc },
  { "/di/#", nano_di },
  { "/dip/#", nano_dip },
  { "/imu", nano_imu },
  { "/imu/acc", nano_imu_acc },
  { "/imu/gyr", nano_imu_gyr },
  { "/pwm/#", nano_pwm },
  { "/do/#", nano_do },
  { "/dac", nano_dac },
#ifdef MOTOR_CARRIER
  { "/nmc/batt", nmc_batt },
  { "/nmc/enc/#", nmc_enc },
  { "/nmc/encvel/#", nmc_encvel },
  { "/nmc/motor/#", nmc_motor },
  { "/nmc/servo/#", nmc_servo },
#endif
  { NULL, NULL }
};
```

All drivers follow a prototype with arguments being a pointer to the table entry and a character buffer.   For example the Nano33's ADC driver function

```
char *nano_adc(Device *dev, char *buf) {
  if (buf == NULL) {
    return dev->channel <= 7 ? (char *)"1024|3.3" : (char *)"?channel";
  } else {
    sprintf(buf, "%d", analogRead(dev->channel));
    return NULL;
  }
}
```
On initialization, when the `inp /adc/2` command is processed the function is called with `buf == NULL` and the function writes a message describing its scaling factors into the passed buffer.  Otherwise it reads the device and formats the value into the passed buffer as an ASCII numeric string.

An example output device driver, for a digital output, is

```
char *nano_do(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 21) return "?channel";
    pinMode(dev->channel, OUTPUT);
  } else {
    digitalWrite(dev->channel, atoi(buf));
  }
  return NULL;
}
```
On initialization, when the `out /do/1` command is processed the function is called with `buf == NULL` and the function checks the validity of the channel number and sets the pin to OUTPUT mode.  Otherwise it parses the ASCII numeric string from the passed buffer and writes it to the pin.

## Command handler

The command handler is driven by a table, new command can be easily added to the table and a new handler function associated with it.
