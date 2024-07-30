# ArduIO

Performing real-time control on a desktop or laptop computer is frequently problematic.  These computers do not have  relevant i/o devices (digital and analog pins) and they run a time-sharing operating system which makes accurate periodic operations very difficult.  Sample jitter, missed samples, and sample rate error are common.  They are, however, very convenient platforms for code development.

Low-cost hardware such as Arduino is available that provides the relevant i/o devices.  Code on the Arduino runs without any operating system using a very fast polling loop.

ArduIO is a way to connect these two types of computing device to allow for convenient and high-quality real-time control using code running on the desktop or laptop.  It comprises two communicating components:

* the i/o _server_ which runs on the Arduino
* the _client_ which runs on a _host_ computer, typically a laptop or desktop with full code development facilities

## Other approaches

There are a number of well known packages that tackle this same problem.  Some of the better known ones
are listed below.

### Firmata

> [Firmata: Towards making microcontrollers act like extensions of the computer,
Hans-Christoph Steiner, NIME09, June 2009.](https://www.nime.org/proceedings/2009/nime2009_125.pdf)

[Firmata](https://github.com/firmata) is shipped with the Arduino IDE.  Firmata uses binary messages based on the [MIDI message format](https://en.wikipedia.org/wiki/MIDI), repurposing MIDI device messages for i/o and making extensive use of SysEx messages. 

Firmata does not really support synchronous sampling.  All inputs are polled at high speed and sent to the client, where subsampling takes place.  This not only wastes communications bandwidth, it also introduces artifacts due to the signal being
resampled. Adding a new type of device is not straightforward, and the client (host-side) software has to know about the different types of device on the server.

Arduino ships with 13 different Firmata sketches, from standard to configurable, and there are 5 other variant libraries that can be installed.
`AnalogFirmata` provides access to analog IO.  It has a fixed sample rate of 20ms

```
  currentMillis = millis();
  if (currentMillis - previousMillis > 20) {
    previousMillis += 20;                   // run this every 20ms
    for (analogPin = 0; analogPin < TOTAL_ANALOG_PINS; analogPin++) {
      if ( analogInputsToReport & (1 << analogPin) )
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
    }
  }
```

Firmata has client libraries written in a diverse range of languages including Python, JS, and Java;
and interfaces to packages such as LabView and Modellica.

PyFirmata is a Python package that runs on the client computer that provides access to the remote i/o.

```
>>> from pyfirmata import Arduino, util
>>> board = Arduino('/dev/tty.usbserial-A6008rIF')
>>> it = util.Iterator(board)
>>> it.start()
>>> board.analog[0].enable_reporting()  # tell the pin to report its values
>>> value = board.analog[0].read()
```

The code `board.analog[0]` returns a `Pin` object which has methods to enable reporting, read a value etc.
A reference to the `Pin` object can also be obtained directly from the `Board` object by
`board.get_pin('a:0:i')` where the string indicates analog, channel 0, input.

The user's code loop must regularly call `board.iterate()` to read the incoming messages or else
setup a background thread using `util.Iterator` to do this.
[pymata-aio](https://pypi.org/project/pymata-aio/) is an alternative approach based on Python asynchronous io.

### Telemetrix

[The Telemetrix Project](https://mryslab.github.io/bots-in-pieces/arduino,stm32,firmata/2020/09/20/telemetrix-phase-1.html), by the author of [pymata-aio](https://pypi.org/project/pymata-aio/), a modern-day replacement for Arduino StandardFirmata, equipped with many more built-in features than StandardFirmata.  It supports many more devices, but not the Nano Motor Carrier which was my main requirement.


### Concerns

* The MIDI protocol and its use of 7-bit binary data adds significant complexity.  Dealing with higher resolution
analog sensors, or large pin numbers is handled with considerable additional complexity.

* Extensive use of enums to specify device types makes expansion more difficult.  Adding a device requires
adding code to host and client, and agreeing on the enum values in both places.

* Capability discovery adds complexity and little benefit.  If the Arduino is used in some real-time system
the connections to the Arduino will have been explictly made and the user code will have to be written taking that
into account.

* Analog and digital i/o are well supported, but more complex devices such as DC motors, encoders and IMUs are not.  They
can be added by writing custom code for the host and Arduino end.

* Firmat's approach to specifying devices, eg. "a:0:i: for analog 0 as input or "d:3:p" for digital pin 3 as PWM is cryptic and limited.

* Firmata and Telemetrix both provide device specific methods in the client classes, eg. `stepper_get_speed()` which means that changes to the server necessitate changes to the API.

* Sampled data may be resampled.

* Relies on the host's clock for sampling (actually the second sampling).


## The ArduIO approach

Elegance and perceptions of complexity are always in the eye of the beholder, but Firmata and Telemetrix, while popular do not have a control engineers approach to the problem.
ArduIO takes a different approach and its design principles are:

* the sampling is paced by a real-time clock on the server device
* minimal complexity protocol, that can be implemented in C and is portable, and is based on compact, but readable, ASCII strings.
* in practice the i/o devices to be read or written at each sample time do not change as the application runs.  This means that they can be declared before the main control loop begins execution, ideally making execution at each sample time more efficient.
* continuous values, such as ADC or DAC values, are sent and received as integers and scaled on the host, removing the need for floating point arithmetic on the Arduino
* the client side API is invariant to the particular Arduino, its i/o devices and shields
* every input device returns scalar or vector value of ints or floats (eg. an IMU accleration measurement), every output device consumes a scalar or vector value of ints or floats 

The current working version of the ArduIO server sketch, which supports analog and digital i/o, and the Nano motor carrier, is under 1,000 lines of commented code.

## Communications protocol

The communications protocol, running over a serial port or WiFi, is readable ASCII text and line oriented.

The protocol is a tradeoff between maximum efficiency (a binary data stream) and a fully self-describing data object.  ArduIO uses a very compact yet still readable ASCII format.  Particular attention is given to the messages transmitted between client and server while a control loop is active, sensor values to the client and output values to the server must be compact if the communications protocol is serial.

The fastest serial transfer rate is 115,200 baud.  With one start and stop bit, that's 11.52kbyte/s.  If the messages were 20 bytes each, the maximum sample rate would be 288Hz. In future it might be worthwhile to adopt a hybrid ASCII/binary protocol where setup uses ASCII data but reading a group of sensors uses a binary protocol.

### Commands

Commands to ArduIO, sent via a serial port or WiFi connection, are remininscent of shell commands, a short command name followed by optional space-separted arguments, followed by a newline (denoted by `↲`). For example:

```
[clear↲] →  [*↲]
[status↲] → [*4|1|||built Jul  3 2024 at 17:23:27↲]
```

The text delimited by `[...]` is a string sent over the "wire", the command sent to the server and the corresponding response.

Commands are terminated by a newline.  Each command responds with a single-line response, possibly just a newline:
* When operating over a serial port, all ArduIO response lines begin with a `*` to distinguish them from debug text that might be sent from the Arduino.
* An ArduIO error is indicated by a line beginning with a `?` followed by single-word error descriptor.
* Numeric data, including sample values, are a single line with fields separated by `|`.  Each input or output device corresponds to a field and is typically a scalar, but for the case of vector valued devices (such as IMUs) the vector elements are separated by `,` within a field.

The server does not prompt or echo typed characters.  Direct control of the server using a terminal emulator is possible, but awkward.

Commands are processed by a table-driven command handler, and commands are dispatched to a handler function.  New commands
are easily added.
The command handler is called repeatedly from the `loop()` function.

| Command   | Description |
| -------   | ------------ |
| `clear`   | clear the input and output tables |
| `status`  | return the size of the tables, WiFi status, build status |
| `inp`     | append entry to the input table |
| `out`     | append entry to the output table |
| `get`     | sample all devices in the input table |
| `set`     | update all devices in the output table |
| `run`     | periodically perform `get` at specified rate |
| `stop`    | stop periodic sampling |
| `blink`   | control server's use of the onboard LED |

## Doing IO

An Arduino supports a large number of i/o capabilities, some of which are referred to simply as _pins_.
We will generalize this and refer to each i/o capability (a digital input or an IMU) as a device.  
Every device is associated with one or more pins on the chip. Some pins are shared between multiple devices, but only only one of those devices can be used at any one time.
Some devices have an associated channel number: channel 2 of the onchip ADC is a different device to channel 3 of the ADC.

### I/O device specification

ArduIO employs a path-like notation for specifying devices.  This means that the client API software is completely agnostic to the devices present on the server and their data types.  An input device is assumed to produce a scalar or vector of ints or floats. An output device accepts a scalar or vector of ints or floats and performs some real-world action.  Whether a device is an input or output is implicit in its name.

Devices at the "root" are onboard, for example on the Nano 33 the devices are:

```
/adc/0   # analog channel 0
/do/13   # digital output 13
/pwm/4   # PWM output channel 4 
/di/2    # digital input 2
/dip/3   # digital input with pullup 3
/imu     # IMU, 6-vector value
/imu/acc # IMU accelerometer, 3-vector value
/imu/gyr # IMU gyroscope, 3-vector value
/dac     # DAC
```

Devices on shields are in a shield-specific "folder".  For example, on the Nano Motor Carrier the devices are:

```
/nmc/adc/0    # analog channel 0
/nmc/enc/1    # encoder channel 1
/nmc/servo/0  # servo channel 0
/nmc/motor/1  # DC motor 1
/nmc/imu/rpy  # IMU orientation, 3-vector value
```

### Input and output tables

In most applications only a subset of available devices are used.  ArduIO uses the concept of an:

*  **input table**, a list on the server of all the devices to be read at each sample time. When the server reads the sensors it sends a delimited string with the values in the order they were added to the input table.
*  **output table**, a list on the server of all the devices to be written at each sample time. When the server writes the sensors it parses a delimited string with the values in the order they were added to the output table.

There is no mechanism to read or write a device without first creating entries in these tables.

A sensor is added to the input table using the `inp` command, and an output device is added to the output table using the `outp` command.  If relevant, these commands can return scale factors as two numbers, the full scale integer value and the corresponding real value. Commands to add i/o devices to the input and output tables look like

```
[inp /di/0↲] →      [*↲]
[inp /di/1↲] →      [*↲]
[inp /adc/2↲] →     [*1024|3.3↲]
[inp /nmc/enc/0↲] → [*↲]
[out /do/13↲] →     [*↲]
```

Note that the third line has returned scale factor information, indicating that this ADC has a maximum value of 1024 (10 bit) which corresponds to 3.3 volts.

The length of the input and output tables is returned as the first two fields of a status message, along
with the build date of the server

```
[status↲] → [*4|1|||built Jul  3 2024 at 17:23:27↲]
```

#### Error checking

The server performs a simple check on the validity of the channel number.

There is no checking for:
* ensuring a pin is used just once
* ensuring that devices which share a common pin are not conflicting (Arduinos overload i/o capability onto pins)
* ensuring that a device is being added to the correct table, ie. an input device must be added to the input table.


### Reading and writing I/O devices

Once the input table has been setup, using the commands above, we can sample _all_ the input devices with a single command

```
[get↲] → [*1|1|288|-63↲]
```

All devices in the input table are read in order and returned in a single line of delimited ASCII text:

- The first field, `1`, corresponds to the first entry in the input table which is for `/di/0`.  `1` in this case means true, the pin has a high voltage.
- The second field, `1`, corresponds to the second entry in the input table which is for `/di/1`.
- The third field, `288`, corresponds to the third entry in the input table which is for `/adc/2`.  288 in this case is the raw unscaled value directly from the ADC.
- The fourth field, `-63`, corresponds to the fourth entry in the input table which is for `/nmc/enc/0`.

Output values can be set in an analagous way

```
[set 1↲] → [↲]
```

which assigns the value of `1` to the only digital output `/do/13`.  If there were multiple devices in the output table the values would be separted by the field delimiter `|`.

The `run` command initiates a periodic process on the server that sends a sample message to the client every sample interval.  For example:

```
[run 100↲] → [*100↲]
 → [*0,0|1|1|290|-63↲]
 → [*1,0|1|1|294|-63↲]
 → [*2,0|1|1|303|-63↲]
 → [*3,0|1|1|307|-63↲]
 → [*4,0|1|1|308|-63↲]
 → [*5,0|1|1|310|-63↲]
 .
 .
 .
```

Every 100ms a new sample is transmitted without being solicited. The first field contains a 2-tuple. The first value is a sequence number which is incremented on successive samples and can be used to determine if samples have been missed.  The second value is the lateness in units of milliseconds.  This is how late the sample was taken, compared to the computed sampling time.  The remaining fields are the values of the four sensors just like the `get` example above.

Commands can be sent to the server while sampling is underway.  The most common such commands are:

* `stop` which stops periodic sampling
* `set` which provides values for all devices in the output table

# Server software

The Arduino sketch lives in the [arduio_server folder](arduio_server).  It is compiled and loaded into the Arduino using the IDE.  The code has various compile options for
debugging, LED blinking, and serial or WiFi operation.  WiFi communications has been found to have quite large and unpredictable latencies.  Serial communication can easily sustain 50Hz control from an iMac connected via USB serial.

# Client software

The client software can be written in any language, but the [reference API implementation is for Python 3](arduio).  An abstract base class `ArduIO` is agnostic to the communications mechanism, whereas the communications specific methods are in the subclasses `ArduIOserial` and `ArduIOwifi`.

For a serially connected arduIO server

```
>>> from arduio import ArduIOserial
>>> a = ArduIOserial("/dev/cu.usbmodem1411101", timeout=1, debug=False)  # connect via serial port
```

The input and output tables are cleared, they may have been set by a previous session
```
>>> a.clear()
```

Next we setup the input and output tables
```
>>> di0 = a.input("/di/0")
>>> di1 = a.input("/di/1")
>>> adc2 = a.input("/adc/2")
>>> nmc_enc0 = a.input("/nmc/enc/0")
>>> led = a.output("/do/13")
```
References to the proxy objects are returned but a list of them can be obtained using the `inputs` and `outputs` property of the server object.  The tables can be displayed

```
>>> print(a)
ArduIO serial server on /dev/cu.usbmodem1411101 @ 115200 baud
  Inputs:
    /di/0
    /di/1
    /adc/2
    /nmc/enc/0
  Outputs:
    /do/13
```

Now that the ArduIO server's input and output tables have been configured, we can request the server to read the input sensors and transmit them by
```
>>> a.receive()
```

The received values are parsed and the proxy objects updated. The sampled values can be displayed
```
>>> print(adc2)
0.928125
>>> print(repr(adc2))
/adc/2 = 0.928125
```
The numeric value, as a float, can be obtained by `adc2.value` or `float(adc2)`. The raw integer value is avaiable by
`int(adc2)`.

By default the server blinks the onboard LED to indicate its state.  Slow blink (1Hz) is idle and processing commands, fast blink (5Hz) is in sampling mode.
If the user wants to control the LED this behaviour can be disabled by
```
>>> a.blink(False)
```

Output proxy objects are updated by the `set` method 

```
led.set(1)  # turn LED on
```

and the value is kept within the proxy object and not immediately transmitted to the output device.  This is done by

```
a.send()  # reflect all output proxy values on the hardware
```

which takes the current value of _all_ output proxy objects and packs them into a single `set` command which is sent to the server.

A very simple control program uses periodic sampling on the ArduIO and a callback function on the client
```
def cb(a, t):
    led.set(int(nmc_enc0) % 2)

>>> a.run(T=30.0, freq=50, callback=cb)
```

The `run` method will execute for 30 seconds at a sample rate of 50Hz.  Every sample time, determined by the ArduIO server the input devices are sampled and transmitted to the client, the proxy input devices are updated, and then the callback function is executed.  The callback function updates proxy values for output device, and after the callback's execution the values of all proxy outputs are collected and sent to the server where the output devices are updated -- the server `send()` method is implicit.

Currently the output devices are updated at some variable time after the inputs were sampled, depending on how long the Python callback function takes to execute.



# Sampling integrity

## Clock skew

Sampling is performed on the Arduino by polling its onboard clock using the `millis()` function which has a 1ms precision.  Clock polling is performed in the `loop()` function which is also polling the input device (serial or WiFi) and blinking the LED.

To eliminate clock skew and rate drift the server mainains the time that the next sample is due. At each sample the due time is incremented by the sample interval.

## Clock jitter

At each `loop()` iteration the current clock time is compared to the next sample time.  If the current time is greater or equal to the due time, a sample is taken.  There is a possibility that other activities in the `loop()` function would make the sample late, so the time between when the sample was taken and when it was due, in milliseconds, is returned as the `late` value in the sample message.


## Missing samples

Each sample message includes a sequence number, starting at zero.  This can be used by the client to determine if a sample message has been lost.

## Heartbeat

Every time a `set` command is received, the server notes the local time.  At every sample time, the time since the last `set` command is computed and if that exceeds two sample times the sampling process is stopped, all output devices are set to zero, and a `?heartbeat` error message is sent to the client.

## Timeout

The client can set a timeout so that if no sample message is received from the server for a period an exception will be thrown.  In practice, if no sample message is received the callback function is not called, the output devices will not be updated, and the heartbeat shutdown procedure will be invoked.

## CRC checks

CRC checks are not implemented in the messages.  For WiFi communications, error checking and recovery will be handled by the TCP/IP stack.  Serial communications is actually performed using virtual UARTS connected by USB, with error checking and recovery performed by the USB stack.


# Future work

* string data?
* handle I2C sensors/outputs
* get WiFi working reliably
* application specific analog scaling
* user specifiable heartbeat action and values
* more comprehensive checking of pin requests, clashes
* report all present devices
* combine inp and out commands into single add
* settable output timeout
* parse arguments in command handler into argc, argv format.
* Additional scaling from volts to application specific units could be performed automatically.
* More comprehensive checking should be in the server-side code, perhaps controlled by a table rather than adhoc logic.
* hold output values in the ArduIO output table and update the output devices immediatly after the sensors are read, ensuring a constant one sample time delay in the control.
