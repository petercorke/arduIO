from arduio import ArduIOserial
import logging

# Format the log message
logging.basicConfig(
    format="%(levelname)s: %(asctime)s.%(msecs)03d -> %(message)s",
    datefmt="%H:%M:%S",
    level=logging.DEBUG,
)

# a = ArduIO("10.0.0.120", 9000, timeout=2, debug=False)
a = ArduIOserial("/dev/cu.usbmodem1411101", timeout=1, debug=False)

a.clear()
print(a.status())

di0 = a.input("/di/0")
di1 = a.input("/di/1")
adc2 = a.input("/adc/2")
# imu_acc = a.input("/imu/acc")
nmc_enc0 = a.input("/nmc/enc/0")
led = a.output("/do/13")

print(a.status())
print(a)


print(repr(adc2))
a.receive()
print(adc2)
print(repr(adc2))

a.blink(False)


def cb(a, t):
    led.set(int(nmc_enc0) % 2)


a.run(T=30.0, freq=50, callback=cb)
