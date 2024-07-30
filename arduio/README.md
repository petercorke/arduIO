This folder holds the Python3 client software, the API.

# Client software

The client software can be written in any language, but the reference impementation is for Python 3.  An abstract base class `ArduIO` is agnostic to the communications mechanism, whereas the communications specific methods are in the subclasses `ArduIOserial` and `ArduIOwifi`.

```
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

A very simple control program uses periodic sampling on the ArduServer and a callback function on the client
```
def cb(a, t):
    led.set(int(nmc_enc0) % 2)

>>> a.run(T=30.0, freq=50, callback=cb)
```

The `run` method will execute for 30 seconds at a sample rate of 50Hz.  Every sample time, determined by the ArduIO server the input devices are sampled and transmitted to the client, the proxy input devices are updated, and then the callback function is executed.  The callback function updates proxy values for output device, and after the callback's execution the values of all proxy outputs are collected and sent to the server where the output devices are updated -- the server `send()` method is implicit.

Currently the output devices are updated at some variable time after the inputs were sampled, depending on how long the Python callback function takes to execute.

FUTURE: hold these in the ArduServer output table and update the output devices immediatly after the sensors are read, ensuring a constant one sample time delay in the control.


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
* hold output values till next sample
* user specifiable heartbeat action and values
* more comprehensive checking of pin requests, clashes
* report all present devices
* combine inp and out commands into single add
* settable output timeout
* parse arguments in command handler into argc, argv format.
