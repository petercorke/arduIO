//=======================================================================================
//  configuration
//=======================================================================================

//#define WIFI
#define SERIAL

#define MOTOR_CARRIER
#define LED
// #define DEBUG

//=======================================================================================
//  includes
//=======================================================================================

#include <Arduino_LSM6DS3.h>

#ifdef MOTOR_CARRIER
#include <ArduinoMotorCarrier.h>
#endif

// TODO:
//  process command handler splits args into an argv style array
//  while sampling, check how long since last write, stop the periodic process, set outputs to specific values
//=======================================================================================
// types
//=======================================================================================
typedef struct _device Device;
typedef char *(*FUNCP)(Device *, char *);

struct _device {
  struct _device *next;
  int channel;
  FUNCP func;
};

// simple linked list functions
struct _list {
  Device *head;
  Device *tail;
};
typedef struct _list List;

void list_append(List *l, Device *d) {
  if (l->tail == NULL) {
    // queue is empty
    l->head = l->tail = d;
  } else {
    l->tail->next = d;
    l->tail = d;
  }
  d->next = NULL;
}

void list_clear(List *l) {
  Device *dev, *next;

  for (dev = l->head; dev != NULL;) {
    next = dev->next;
    free(dev);
    dev = next;
  }
  l->head = l->tail = NULL;
}

int list_len(List *l) {
  Device *dev;
  int n = 0;

  for (dev = l->head; dev != NULL; dev = dev->next)
    n++;

  return n;
}

List ilist;
List olist;


typedef struct _nf {
  char *name;
  FUNCP func;
} NF;


//=======================================================================================
// globals
//=======================================================================================

#ifdef WIFI
// network
#include <WiFiNINA.h>
WiFiServer server(9000);
WiFiClient client;
int connected = false;
int status = WL_IDLE_STATUS;

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
// char ssid[] = "";     // your network SSID (name)
// char pass[] = "";  // your network password (use for WPA, or use as key for WEP)

int keyIndex = 0;             // your network key index number (needed only for WEP)
#endif

// led blinking
#ifdef LED
int ledBlink = true;     // enable led blinking by server
uint intervalLED = 500;  // server blink interval in ms
unsigned long nextLED;   // time of next toggle in ms
#endif

// periodic sampling
uint intervalSample = 0;    // sample interval in ms
unsigned long nextSample;   // time of next sample in ms
unsigned int nsamples = 0;  // sample number in current run

unsigned long last_set;    // time of the last set operation


char *id = "built " __DATE__ " at " __TIME__;

// const char * add_io(List *list, char *path);


//=======================================================================================
//  WiFi setup
//=======================================================================================



void setup() {
#ifdef LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#endif

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("@@@@ ArduIO server - ");
#if defined(WIFI)
  Serial.print("WiFi: ");
#elif defined(SERIAL)
  Serial.print("Serial: ");
#endif

  Serial.print(id);
#ifndef DEBUG
  Serial.print(" - serial debug is not enabled");
#endif
  Serial.println("");

  if (!IMU.begin()) {
    Serial.println("LSM6DS3");
    while (1)
      ;
  }

#ifdef WIFI
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print(">>>> Connecting to ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
#endif

#ifdef LED
  digitalWrite(LED_BUILTIN, LOW);
#endif
}

#ifdef WIFI
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("  IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("  signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
#endif

char ibuf[80];

int linenum;
int in_count = 0;

unsigned long loop_count = 0;


void showstuff(unsigned long t, int x1, int x2, char *message) {
  if (t < 200)
    return;
    
  Serial.print(" excessive timein server.available()");
  Serial.print(message);
  Serial.print(": ");
  Serial.print(t);
  Serial.print(", flag = ");
  Serial.print(x1);
  Serial.print(", ");
  Serial.print(x2);
  Serial.print(", count = ");
  Serial.print(loop_count);
  Serial.print("\n");
}


void loop() {
  unsigned long t0 = millis();
  int flag = 0;

  loop_count++;

#if defined(WIFI)
  int status_old = server.status();
  client = server.available();

  unsigned long tx = millis();
  showstuff(tx - t0, server.status(), status_old, "x");

  // attempt to read characters from socket and process the command
  if (client && (client.available() > 0)) {
    char c = client.read();
#elif defined(SERIAL)
  // attempt to read characters from serial port and process the command
  if (Serial.available() > 0) {
    char c = Serial.read();
#endif

  // unsigned long t1 = millis();
  // showstuff(t1 - t0, flag, "1");

#ifdef DEBUG
    Serial.write(c);
#endif

    if (c != '\n') {
      // stash the char
      if (in_count > 75) {Serial.print("buf index too big "); Serial.println(in_count);}
      ibuf[in_count++] = c;
    } else {
      // end of line, process the command
      ibuf[in_count] = '\0';  // terminate the string
      in_count = 0;

  unsigned long t4a = millis();


      process_command(ibuf);

    //   unsigned long t4b = millis();
    // showstuff(t4b - t4a, flag, "4");
      flag = 1;
    }
  }

  // unsigned long t2 = millis();
  // showstuff(t2 - t0, flag, "2");

  // -------------------  periodic "tasks"

  // upload the data
  if (intervalSample > 0)
    periodic(intervalSample, nextSample, send_data, (void *)0);

  // blink the led
  periodic(intervalLED, nextLED, toggle_led, (void *)0);

  // unsigned long t3 = millis();
  // showstuff(t3 - t0, flag, "3");
}


//=======================================================================================
//  command handler
//=======================================================================================

typedef struct _command {
  char *cmd;
  void (*func)(char *, char *);
} COMMAND;

COMMAND commands[] = {
  { "get", cmd_get },
  { "set", cmd_set },
  { "clear", cmd_clear },
  { "inp", cmd_inp },
  { "out", cmd_out },
  { "run", cmd_run },
  { "stop", cmd_stop },
  { "status", cmd_status },
  { "blink", cmd_blink },
  { NULL, NULL }
};

// command is the keyword, or the keyword followed by a single space followed by optional arguments

#define OBUFLEN 80

int sampling;

void process_command(char *cmd) {
  char obuf[256];
  char tmp[OBUFLEN];

#ifdef DEBUG
  Serial.print("line #");
  Serial.print(linenum);
  Serial.print(": ");
  Serial.println(ibuf);
#endif
  linenum++;
// #if defined(WIFI)
//   strcpy(obuf, "\n");  // default response is empty string
// #elif defined(SERIAL)
//   strcpy(obuf, "");  // default response is empty string
// #endif

  for (COMMAND *c = commands; c->cmd != NULL; c++) {
    if (strncmp(cmd, c->cmd, strlen(c->cmd)) == 0) {
      // matching command
      char *sp = strchr(cmd, ' '), *p;
      if (sp == NULL)
        p = NULL;  // no space
      else
        p = sp + 1;  // following arguments

      sampling = intervalSample > 0;
      strcpy(obuf, "*");  // default return is an empty record, can be overwritten
      (*(c->func))(obuf+1, p);
#if defined(WIFI)
      client.print(obuf);
#ifdef DEBUG
      Serial.println(obuf);
#endif
#elif defined(SERIAL)
if (!sampling) {
    strcat(obuf, "\n");
    Serial.write(obuf);  // send response so long as not in sampling mode
}
#endif
      return;
    }
  }
  // unknown command
#if defined(WIFI)
  client.print("?cmd\n");
#elif defined(SERIAL)
  Serial.print("?cmd");
#endif
#ifdef DEBUG
  Serial.print("bad command");
  Serial.println(cmd);
#endif
  return;
}

void cmd_get(char *out, char *args) {
  read_sensors(out);
}

void cmd_set(char *out, char *args) {
  write_outputs(args);
  last_set = millis();
  strcpy(out, "");
}

void cmd_clear(char *out, char *args) {
  list_clear(&ilist);
  list_clear(&olist);
  ledBlink = true;
}

void cmd_inp(char *out, char *args) {
  const char *ret = add_io(&ilist, args);
  if (ret != NULL) {
    strcpy(out, ret);
    // strcat(out, "\n");
  }
}

void cmd_out(char *out, char *args) {
  const char *ret = add_io(&olist, args);
  if (ret != NULL) {
    strcpy(out, ret);
    // strcat(out, "\n");
  }
}

void cmd_run(char *out, char *args) {
  nsamples = 0;
  nextSample = 0;
  intervalSample = atoi(args);
  intervalLED = 100;
  last_set = millis();
#ifdef DEBUG
  Serial.print("start run ");
  Serial.println(intervalSample);
#endif
  snprintf(out, OBUFLEN, "%d", intervalSample);  // send back the interval
}

void cmd_stop(char *out, char *args) {
  intervalSample = 0;
  intervalLED = 500;
  sampling = 0;
#ifdef DEBUG
  Serial.println("stopping");
#endif
}

void cmd_status(char *out, char *args) {
  int ninp = list_len(&ilist);
  int nout = list_len(&olist);

#if defined(WIFI)
  IPAddress ip = WiFi.localIP();
  long rssi = WiFi.RSSI();

  snprintf(out, OBUFLEN, "%d|%d|%d.%d.%d.%d|%d|%s", ninp, nout, ip[0], ip[1], ip[2], ip[3], rssi, id);
#elif defined(SERIAL)
  snprintf(out, OBUFLEN, "%d|%d|||%s", ninp, nout, id);
#endif

#ifdef DEBUG
  Serial.println(out);
#endif
}

void cmd_blink(char *out, char *args) {
#ifdef LED
  ledBlink = atoi(args);
// Serial.print("blink: "); Serial.println(ledBlink);
#endif
}


//=======================================================================================
//  periodic processes
//=======================================================================================

void periodic(uint interval, unsigned long &next, void (*function)(void *, int), void *arg) {
  unsigned long tnow = millis();

  if (next == 0)
    next = tnow;  // value was uninitialized

  if (tnow >= next) {
    (*function)(arg, (int)(tnow - next));  // do the next periodic operation

    // update the time for the next operation, this way stops drift
    // and skips missed samples
    while (tnow >= next)
      next += interval;
  }
}

void send_data(void *arg, int late) {
  char obuf[256];
  unsigned long tnow = millis();

  // check if outputs are being updated
  if ((nsamples > 0) && ((tnow - last_set) > (2 * intervalSample))) {
    intervalSample = 0;  // stop further sampling
    zero_outputs();      // make outputs safe
#if defined(WIFI)
    server.print("?heartbeat\n");    //report it to the host
#elif defined(SERIAL)
    Serial.print("?heartbeat\n");    //report it to the host
#endif
    Serial.print("@@@ no heartbeat "); Serial.print(nsamples); Serial.print("; since last write = "); Serial.print(tnow-last_set); 
    Serial.print("; late = "); Serial.println(late); 
    return;
  }

  //line starts with sequence number and lateness in milliseconds
  sprintf(obuf, "*%d,%d|", nsamples++, late);

  // then the data values
  read_sensors(obuf + strlen(obuf));
  strcat(obuf, "\n");

#if defined(WIFI)
  server.print(obuf);
#elif defined(SERIAL)
  Serial.write(obuf);
#endif

//   unsigned long t2 = millis();

//   if ((t2 - tnow) > 200) {Serial.print("excessive send_data() time: "); Serial.println(t2 - tnow);}
// #ifdef DEBUG
//   Serial.print("sample: ");
//   Serial.print(client.connected());
//   Serial.print(": ");
//   Serial.print(obuf);
// #endif
}

void toggle_led(void *arg, int late) {
  static int ledState = LOW;

  // if the LED is off turn it on and vice-versa:
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
// set the LED with the ledState of the variable:
#ifdef LED
  if (ledBlink)
    digitalWrite(LED_BUILTIN, ledState);
#endif
}

//=======================================================================================
//  sensor handling
//=======================================================================================

void read_sensors(char *buf) {
  char *t;
  Device *dev;
  int len;

  for (dev = ilist.head, t = buf; dev != NULL; dev = dev->next) {
    (*dev->func)(dev, t);
    if (dev != ilist.tail) {
      // not the last entry, add a separator and bump the pointer
      len = strlen(t);
      t[len] = '|';
      t += len + 1;
    }
  }
}

void write_outputs(char *cmd) {
  Device *dev;
  char *p;

  // Serial.print("write outputs: "); Serial.println(cmd);
  for (dev = olist.head; dev != NULL; dev = dev->next) {
    if (dev != olist.tail) {
      for (p = cmd; *p != ','; p++)
        ;
      // p points to terminating comma
    }
    (*dev->func)(dev, cmd);
    cmd = p + 1;  // point to next field or eos
  }
}

void zero_outputs() {
  Device *dev;

  // Serial.print("write outputs: "); Serial.println(cmd);
  for (dev = olist.head; dev != NULL; dev = dev->next) {
    (*dev->func)(dev, "0");
  }
}

// /adc/0
// /imu
// /nmc/adc/0
// /nmc/imu

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

const char *add_io(List *list, char *path) {
  int channel = -1;
  Device *d = NULL;
  char *retval;

  for (NF *dev = nano; dev != NULL; dev++) {
    // for each device in the table
    char *p, *q;
    for (p = dev->name, q = path; *p != '\0' && *q != '\0'; p++, q++) {
      if (*p == '#') {
        channel = atoi(q);
        break;
      } else if (*p != *q) {
        goto nextdevice;
      }
    }
    // we have a match, create a Device object and out it in the list
    d = (Device *)malloc(sizeof(Device));
    d->channel = channel;
    d->func = dev->func;

    retval = (*(dev->func))(d, NULL);
    list_append(list, d);  // add device to input or output list
    return retval;         // return the device's initialization string

nextdevice:;
  }
  return "?badname";  // unknown device
}

//=======================================================================
// nano i/o device handlers
//=======================================================================

// sensors

// built in ADC, 12 bit: /adc/0 to /adc/7, overlaps digital 14-21
// built in DI: /di/0 to /di/21
// built in DI with pullup: /dip/0 to /dip/21
// built in DO: /do/0 to /do/21
// built in DAC, 10 bit: /dac

// analogRead
// analogWrite  pinMode(OUTPUT)   INPUT, OUTPUT, or INPUT_PULLUP
// map(sensorValue, 0, 1023, 0, 255);

char *nano_adc(Device *dev, char *buf) {
  if (buf == NULL) {
    return dev->channel <= 7 ? (char *)"1024|3.3" : (char *)"?channel";
  } else {
    sprintf(buf, "%d", analogRead(dev->channel));
    return NULL;
  }
}

char *nano_di(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 21) return "?channel";
    pinMode(dev->channel, INPUT);
  } else {
    sprintf(buf, "%d", digitalRead(dev->channel));
  }
  return NULL;
}

char *nano_dip(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 21) return "?channel";
    pinMode(dev->channel, INPUT_PULLUP);
  } else {
    sprintf(buf, "%d", digitalRead(dev->channel));
  }
  return NULL;
}

char *nano_imu(Device *dev, char *buf) {
  if (buf != NULL) {
    nano_imu_acc(dev, buf);
    nano_imu_gyr(dev, buf);
  }
  return NULL;
}

char *nano_imu_acc(Device *dev, char *buf) {
  if (buf != NULL) {
    float x, y, z;

    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      sprintf(buf, "%f,%f,%f", x, y, z);
    } else {
      strcpy(buf, "NaN,NaN,NaN");
    }
  }
  return NULL;
}

char *nano_imu_gyr(Device *dev, char *buf) {
  if (buf != NULL) {
    float x, y, z;

    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      printf(buf, "%f,%f,%f", x, y, z);
    } else {
      strcpy(buf, "NaN,NaN,NaN");
    }
  }
  return NULL;
}

char *nano_do(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 21) return "?channel";
    // Serial.print("do init: "); Serial.println(dev->channel);

    pinMode(dev->channel, OUTPUT);
  } else {
    // Serial.print("do: "); Serial.print(dev->channel); Serial.print(" ,"); Serial.println(buf);
    digitalWrite(dev->channel, atoi(buf));
  }
  return NULL;
}

char *nano_pwm(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 13) return "?channel";
  } else {
    analogWrite(dev->channel, atoi(buf));
  }
  return NULL;
}

char *nano_dac(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 1) return "?channel";
  } else {
    analogWrite(A0, atoi(buf));
  }
  return NULL;
}

#ifdef MOTOR_CARRIER

// motor carrier
// DC motors: M1 to M4
// servo motors: servo1 to servo 4
// encoders: encoder1 to encoder2
// battery: battery
// motor controller: controller
// PID controllers: pid1 to pid2

char *nmc_batt(Device *dev, char *buf) {
  if (buf != NULL)
    sprintf(buf, "%.2f", battery.getConverted());
  return NULL;
}

char *nmc_enc(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 1) return "?channel";
  } else {
    int v;

    switch (dev->channel) {
      case 0: v = encoder1.getRawCount(); break;
      case 1: v = encoder2.getRawCount(); break;
    }
    sprintf(buf, "%d", v);
  }
  return NULL;
}

char *nmc_encvel(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 1) return "?channel";
  } else {
    int v;

    switch (dev->channel) {
      case 0: v = encoder1.getCountPerSecond(); break;
      case 1: v = encoder2.getCountPerSecond(); break;
    }
    sprintf(buf, "%d", v);
  }
  return NULL;
}

char *nmc_motor(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 3) return "?channel";
  } else {
    int v = atoi(buf);  // duty cycle percentage in range [-100, 100]

    switch (dev->channel) {
      case 0: M1.setDuty(v); break;
      case 1: M2.setDuty(v); break;
      case 2: M3.setDuty(v); break;
      case 3: M4.setDuty(v); break;
    }
  }
  return NULL;
}

char *nmc_servo(Device *dev, char *buf) {
  if (buf == NULL) {
    if (dev->channel > 3) return "?channel";
  } else {
    int v = atoi(buf);  // servo angle in range [0, 180]

    switch (dev->channel) {
      case 0: servo1.setAngle(v); break;
      case 1: servo2.setAngle(v); break;
      case 2: servo3.setAngle(v); break;
      case 3: servo4.setAngle(v); break;
    }
  }
  return NULL;
}
#endif

// check out https://github.com/arduino-libraries/ArduinoMotorCarrier/tree/master/examples/NanoMotorCarrier/IMU_Test
// void nmc_read_imu(char *buf, int16_t channel) {}
// void nmc_read_imu_acc(char *buf, int16_t channel) {}
// void nmc_read_imu_gyr(char *buf, int16_t channel) {}
// void nmc_read_imu_mag(char *buf, int16_t channel) {}
// void nmc_read_imu_rpy(char *buf, int16_t channel) {}
