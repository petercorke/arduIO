import sys
import numpy as np
import time
import socket
import serial
import logging


def _format_ctl(s):
    s = s.replace("\n", "↲")
    s = s.replace("\r", "\\r")
    return s


class ArduIO:

    class Input:
        def __init__(self, path, response):
            """Object representing a physical input device on the ArduIO server

            :param path: Path name of the sensor
            :type path: str
            :param response: string returned by Arduino server, empty string if no scaling
            :type response: str
            """
            self._path = path
            self.value = None

            if response != "":
                a, b = map(float, response.split("|"))
                self.scale = b / a
                self.offset = 0
            self._response = response

        def set(self, value):
            """Assign a value to the sensor input

            :param value: The value returned by the ArduIO server
            :type value: str

            The value returned on the wire is saved in the local object which
            represents it.  Analogy values are optionall scaled and offset.
            """
            if self._response:
                self.raw = value
                self.value = float(value) * self.scale + self.offset
            else:
                self.raw = value
                self.value = value

        def __int__(self):
            """Get the integer value of the sensor input

            The raw value is returned, not the scaled value.
            """
            return int(self.raw)

        def __float__(self):
            """Get the float value of the sensor input

            If the sensor has a scale and offset, the scaled value is returned.
            """
            return float(self.value)

        def __str__(self):
            """Get the string value of the sensor input"""
            return str(self.value)

        def __repr__(self):
            return f"{self.path} = {self.value}"

        @property
        def path(self):
            """Get the path name of the input device"""
            return self._path

    class Output:
        def __init__(self, path, response):
            """Object representing a physical output device on the ArduIO server

            :param path: Path name of the sensor
            :type path: str
            :param response: string returned by Arduino server, empty string if no scaling
            :type response: str
            """
            self._path = path
            self._response = response
            self.value = None

        def set(self, value):
            """Set value of the output

            :param: value: The value to be sent to the ArduIO server
            :type value: int or float

            Note that the value is not sent to the ArduIO server until the send() method is called.
            """
            self.value = value

        def __int__(self):
            return int(self.value)

        def __float__(self):
            return float(self.value)

        def __str__(self):
            return str(self.value)

        def __repr__(self):
            return f"{self.path} = {self.value}"

        @property
        def path(self):
            """Get the path name of the output device"""
            return self._path

    def __init__(self, debug=False):
        """Connect to an ArduServer

        :param host: internet address of the ArduIO server
        :type host: str
        :param port: internet port of the ArduIO server, defaults to 9000
        :type port: int, optional
        :param debug: display messages to/from the ArduIO server, defaults to False
        :type debug: bool, optional
        :param timeout: ArduIO message receive timeout in seconds, defaults to None
        :type timeout: float, optional
        """
        self._debug = debug
        self._inputs = []
        self._outputs = []
        self._buf = ""
        self._sendtime = []
        self._sendlast = None

        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.DEBUG)

        # create console handler and set level to debug
        ch = logging.StreamHandler()

        # create formatter
        formatter = logging.Formatter(
            "%(levelname)s: %(asctime)s.%(msecs)03d -> %(message)s",
            datefmt="%H:%M:%S",
        )

        # add formatter to ch
        ch.setFormatter(formatter)

        # add ch to logger
        self._logger.addHandler(ch)

    def log(self, msg):
        """Send a message to the ArduIO server log

        :param msg: Message to be logged
        :type msg: str
        """
        self._logger.info(msg)

    def __str__(self):
        """Convert the ArduIO object to a string

        This shows the details of the input and output tables as held by the
        ArduIO object.

        :return: String representation of the ArduIO object
        :rtype: str
        """
        s = ""
        if len(self.inputs) > 0:
            s += "\n  Inputs:"
            for inp in self.inputs:
                s += f"\n    {inp.path}"
        if len(self.outputs) > 0:
            s += "\n  Outputs:"
            for outp in self.outputs:
                s += f"\n    {outp.path}"
        return s

    def input(self, path):
        """Add an input to the remote ArduIO server

        :param path: sensor pathname
        :type path: str
        :raises Exception: unknown sensor path
        :return: Object representing the sensor input
        :rtype: Input

        Sensor names are pathlike descriptors, for example
        ```
        /adc/0
        /imu
        /imu/acc
        /nmc/encoder/0
        ```
        The last two examples are for sensors on a Nano Motor Control board. Some sensors
        may have multiple channels, for example digital inputs or ADCs.

        Channel numbers are often Arduino pin numbers, but the exact capability of a pin
        depends on the hardware configuration.

        A sensor may be a scalar (int or float value) or vector-valued (list of int or float values).
        For example `/imu` returns a 6 values, while `/imu/acc` returns 3 values.

        If the sensor path is unknown, or the channel number is invalid an exception is raised.

        All inputs are added to the scan list on the ArduIO server, and are read by the get() method.
        """
        response = self._send(f"inp {path}")
        if response[0] == "?":
            raise Exception(f"Invalid input path: {path}")

        inp = ArduIO.Input(path, response.strip())
        self._inputs.append(inp)
        return inp

    def output(self, path):
        """Add an output to the remote ArduIO server

        :param path: output pathname
        :type path: str
        :raises Exception: unknown output path
        :return: Object representing the output signal
        :rtype: Input

        Output names are pathlike descriptors, for example
        ```
        /do/0
        /dac
        /nmc/motor/2
        ```
        The last example is for a motor drive output on a Nano Motor Control board. Some outputs
        may have multiple channels, for example digital outputs, PWMs or DACs.

        Channel numbers are often Arduino pin numbers, but the exact capability of a pin
        depends on the hardware configuration.

        If the sensor path is unknown, or the channel number is invalid an exception is raised.
        """
        response = self._send(f"out {path}")
        if response[0] == "?":
            raise Exception(f"Invalid input path: {path}")
        out = ArduIO.Output(path, response.strip())
        self._outputs.append(out)
        return out

    @property
    def inputs(self):
        """List of all sensor inputs added to the ArduIO server

        :return: All added sensor inputs
        :rtype: list of Input objects
        """
        return self._inputs

    @property
    def outputs(self):
        """List of all outputs added to the ArduIO server

        :return: All added outputs
        :rtype: list of Output objects
        """
        return self._outputs

    def clear(self):
        """Clear all sensor inputs and outputs from the ArduIO server"""
        response = self._send("clear")
        self.inputs.clear()
        self.outputs.clear()

    def receive(self):
        """Receive sensor inputs from ArduServer

        Read all sensor inputs from the ArduServer and update the corresponding Input objects
        """
        response = self._send("get")
        if self._debug:
            self._logger.debug(f"  [recv] --> [{_format_ctl(response)}]")

        data = response.split("|")
        for value, inp in zip(data, self.inputs):
            inp.set(value)

    def send(self):
        """Send output values to ArduServer

        Copy all output values from the corresponding Ouput objects to the ArduServer
        outputs.
        """
        values = [str(outp) for outp in self.outputs]
        response = self._send("set " + "|".join(values), retval=False)

    def blink(self, enable):
        """Enable or disable the LED blink on the ArduIO server

        :param enable: Control LED blinking
        :type enable: bool

        By default the LED on the ArduIO server blinks at 1 Hz and faster when it is
        sampling.  This can be disabled by setting enable to False, enabling the LED
        to be used by the user application.
        """
        response = self._send(f"blink {int(enable)}")

    # def getline(self):
    #     if self._buf:
    #         line, self._buf = self._buf.split("\n", 1)
    #         return line
    #     else:
    #         response = self.s.recv(1024).decode("utf-8")
    #         line, self._buf = self._buf.split("\n", 1)
    #         return line

    def run(self, T=None, dt=None, freq=None, callback=None, spinner=False):
        """Run the ArduIO server for a specified time or number of steps

        :param T: Time to run the server in seconds or number of steps
        :type T: float or int
        :param dt: Sampling time interval in seconds
        :type dt: float
        :param freq: Sampling frequency in Hz
        :type freq: float
        :param callback: Function to be called at each time step
        :type callback: callable
        :param spinner: Display a spinner while running
        :type spinner: bool

        The ArduIO server is run for a specified time or number of steps.  The ArduIO server
        is started with the specified sampling time interval, and the callback function
        is called at each time step.  The callback function is expected to update the ``Output``
        objects, and those values are sent to the ArduIO server after the callback finishes.

        .. note::

            * Only one of ``dt``, or ``freq`` may be specified.
            * If ``T`` is an int it is interpreted as the number of steps, if it is a float
              it is interpreted as the time in seconds.

        """

        if dt is None and freq is None:
            raise Exception("Either dt or freq must be specified")
        elif freq:
            dt = 1 / freq
        dt_ms = int(dt * 1000)
        dt = dt_ms / 1000

        response = self._send(f"run {dt_ms}")
        if self._debug:
            self._logger.debug(f"start run {response}")

        self.late = []
        self.badseq = []
        self.t_callback = []
        self.seq = -1

        if type(T) == int:
            n = T
        elif type(T) == float:
            n = int(T / dt)
        else:
            raise Exception("Invalid type for T")

        print(f"Running for {n} steps at {dt} s per step")

        last_response = "foo"
        for i in range(n):
            response = self.read()

            last_response = response

            # optionally display the response, debug only
            # if self._debug:
            #     self._logger.debug(f"  [recv] --> [{format_ctl(response)}]")

            # check for mutiple lines in response
            if response.count("\n") > 1:
                self._logger.warning(
                    f"multiple lines in response: {_format_ctl(response)}"
                )

            if response == "\n":
                continue

            # check for error response from ArduIO server
            if response[0] == "?":
                self._logger.warning(f"Error: ArduIO server: {response}")
                raise Exception(f"Error: ArduIO server: {response}")

            # deal with the message from ArudIO server, these come periodically and not
            # in response to a command
            data = response.split("|")
            seq, late = map(int, data[0].split(","))
            if late > 0:
                self.late.append(int(late))
                self._logger.warning(f"Warning: late by {late} @ {seq}")
            if seq != self.seq + 1:
                self.badseq.append((self.seq, seq))
                self._logger.warning(f"Error: expected {self.seq+1}, got {seq}")
            self.seq = seq

            for value, inp in zip(data[1:], self.inputs):
                inp.set(value)

            if callback:
                t0 = time.time()
                callback(self, i * dt)
                self.send()
                t1 = time.time()
                self.t_callback.append(t1 - t0)

            if spinner:
                sys.stdout.write("-\|/"[i % 4] + "\b")
                sys.stdout.flush()

        if self._debug:
            self._logger.debug("end of loop")
        try:
            response = self._send("stop")
        except socket.timeout:
            self._logger.warning("stop timeout")

        # report performance measures
        if self.late:
            print(f"Number late:  {len(self.late)}")
            print(f"Average late: {np.mean(self.late)}")
            print(f"Max late:     {max(self.late)}")

        if self.badseq:
            print(f"Number badseq: {len(self.badseq)}")
            print(f"Bad sequences: {self.badseq}")
            maxerr = max([(seq - exp) for exp, seq in self.badseq])

        if callback:
            print("Callback times:")
            print(f"  average:            {np.mean(self.t_callback)*1000:.1f} ms")
            print(f"  standard deviation: {np.std(self.t_callback)*1000:.1f} ms")
            print(f"  maximum:            {max(self.t_callback)*1000:.1f} ms")
            print(f"  number overlong:    {sum([t > dt for t in self.t_callback])}")


class ArduIOwifi(ArduIO):
    def __init__(self, host, port=9000, debug=False, timeout=None):
        """Connect to an ArduServer

        :param host: internet address of the ArduIO server
        :type host: str
        :param port: internet port of the ArduIO server, defaults to 9000
        :type port: int, optional
        :param debug: display messages to/from the ArduIO server, defaults to False
        :type debug: bool, optional
        :param timeout: ArduIO message receive timeout in seconds, defaults to None
        :type timeout: float, optional
        """
        super().__init__(debug)
        self.host = host
        self.port = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if timeout:
            self.s.settimeout(timeout)

    def __str__(self):
        s = f"ArduIO WiFi server at {self.host}:{self.port}"
        s += super().__str__()
        return s

    def _send(self, cmd, retval=True):
        self.s.send(bytes(cmd + "\n", "utf-8"))
        tnow = time.time()
        self._sendtime.append(tnow)
        self._sendlast = tnow
        if not retval:
            return None
        # nextline may raise a socket.TimeoutError
        response = self.s.recv(1024).decode("utf-8")
        if response[0] == "?":
            raise Exception(f"Error: ArduIO server: {cmd} returned {response}")
        if self._debug:
            print(f"  [{cmd}] --> [{_format_ctl(response)}]")
        return response

    def read(self):
        """Read a line from the ArduIO server

        :return: Line of text from the ArduIO server, including the trailing '\n'
        :rtype: str
        """

        try:
            response = self.s.recv(1024).decode("utf-8")
        except socket.timeout as e:
            # handle read timeout
            self._logger.warning(f"Timeout error")
            self._logger.warning(f"  {last_response = }")
            self._logger.warning(f"  {i = }")
            self._logger.warning(f"  {self.late = }")
            self._logger.warning(f"  {self.badseq = }")
            self._logger.warning(f"  last send - now {self._sendlast - time.time()}")
            self._logger.warning(f"  {np.diff(self._sendtime)[-10:]}")
            raise e
        return response

    def status(self):
        """Get the status of the ArduIO server

        :return: Status of the ArduIO server
        :rtype: str

        Returns a tuple describing the status of the ArduIO server: the number of inputs,
        the number of outputs, the IP address, the RSSI, and the build version
        """
        response = self._send("status")
        status = response.split("|")
        return [
            int(status[0]),
            int(status[1]),
            status[2],
            int(status[3]),
            status[4].rstrip(),
        ]


class ArduIOserial(ArduIO):
    def __init__(self, port, baudrate=115200, debug=False, timeout=None):
        """Connect to an ArduServer

        :param host: internet address of the ArduIO server
        :type host: str
        :param port: internet port of the ArduIO server, defaults to 9000
        :type port: int, optional
        :param debug: display messages to/from the ArduIO server, defaults to False
        :type debug: bool, optional
        :param timeout: ArduIO message receive timeout in seconds, defaults to None
        :type timeout: float, optional
        """
        super().__init__(debug)
        self.port = port
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )  # open serial port
        self.ser.reset_input_buffer()

    def __str__(self):
        s = f"ArduIO serial server on {self.port} @ {self.ser.baudrate} baud"
        s += super().__str__()
        return s

    def _send(self, cmd, retval=True):
        self.ser.write(bytes(cmd + "\n", "utf-8"))
        tnow = time.time()
        self._sendtime.append(tnow)
        self._sendlast = tnow
        if not retval:
            return None
        while True:
            response = self.ser.read_until(b"\n").decode("utf-8")
            if response[0] == "*":
                if self._debug:
                    self._logger.debug(f"  [{cmd}] --> [{_format_ctl(response)}]")
                return response[1:]
            elif response[0] == "?":
                raise Exception(f"Error: ArduIO server: {cmd} returned {response}")
            else:
                if response == "\n":
                    print("NL")
                else:
                    print(response)  # debug output from the ArduIO server

    def read(self):
        """Read a line from the ArduIO server

        :return: Line of text from the ArduIO server, including the trailing '\n' and excluding the leading '*'
        :rtype: str
        """
        response = self.ser.read_until(b"\n").decode("utf-8")
        if self._debug:
            self._logger.debug(f"  [recv] --> [{_format_ctl(response)}]")
        return response[1:]

    def status(self):
        """Get the status of the ArduIO server

        :return: Status of the ArduIO server
        :rtype: str

        Returns a tuple describing the status of the ArduIO server: the number of inputs,
        the number of outputs, the IP address, the RSSI, and the build version
        """
        response = self._send("status")
        status = response.split("|")
        return [int(status[0]), int(status[1]), None, None, status[4].rstrip()]
        return status
