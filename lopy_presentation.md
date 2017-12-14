autoscale: true


## Micropython
#### (and the LoPy Microcontroller)

<br>
<br>
<br>
<br>
<br>
<br>
######This presentation was created using [Deckset](https://itunes.apple.com/us/app/deckset/id847496013?mt=12)

---

## What is MicroPython?

- Lean/efficient Python3
- interactive prompt
- arbitrary precision integers
- closures
- list comprehension
- generators
- exception handling

---

## Micropython Hardware Landscape

| Hardware |
| --- |
| Pyboard (Cortex-M4F) |
| WIPY 3.0, LoPy, SiPy, LoPy 4, Guy, FiPy (ESP32) |
| ESP8266 Chips |
| Unix Port |

---

## Micropython Software Landscape
- Open Source
- Main: [https://github.com/micropython/micropython](https://github.com/micropython/micropython)
- ESP32 Port: [https://github.com/micropython/micropython-esp32](https://github.com/micropython/micropython-esp32)
- Adafruit has the `CircuitPython` fork for their microcontrollers: [https://github.com/adafruit/circuitpython](https://github.com/adafruit/circuitpython)

Late Breaking News... ESP32 was just merged upstream to the micropython main branch!

---

#LoPy by Pycom

![inline 90%](https://pycom.io/wp-content/uploads/2017/08/lopyDiagram4-01.png )

---

## LoPy Features

- Powerful CPU, BLE and state of the art WiFi radio. 1KM Wifi
Range
- Can also double up as a Nano LoRa gateway
- MicroPython enabled
- Fits in a standard breadboard (with headers)
- Ultra-low power usage: a fraction compared to other connected
micro controllers

---

## LoPy Processing
- Espressif ESP32 chipset
- Dual processor + WiFi radio System on Chip.
- Network processor handles the WiFi connectivity and the IPv6
stack.
- Main processor is entirely free to run the user application.
- An extra ULP-coprocessor that can monitor GPIOs, the ADC
channels and control most of the internal peripherals during
deep-sleep mode while only consuming 25uA.

---

| LoPy Interfaces | LoPy Power |
| --- | --- |
| 2 x UART, 2 x SPI, I2C, I2S, micro SD card | Input: 3.3V - 5.5V |
| Analog channels: 8x12 bit ADCs | 3v3 output capable of sourcing up to 400mA |
| Timers: 4x16 bit with PWM and input capture | WiFi: 12mA in active mode, 5uA in standby |
| DMA on all peripherals | Lora: 15mA in active mode, 10uA in standby |
| GPIO: Up to 24 | |

---

| LoPy Memory | RTC |
| --- | --- |
| RAM: 512KB | Running at 32KHz |
| External flash 4MB | |
| Hardware floating point acceleration | |
| Python multi-threading | |

---

| LoPy Security & Certifications | Hash / encryption |
| --- | --- |
| SSL/TLS support | SHA, MD5, DES, AES |
| WPA Enterprise security | |
| FCC - 2AJMTWIPY2R | |
| CE 0700 | |

---

[LoPy Pinout](https://docs.pycom.io/chapter/datasheets/development/lopy.html)
![inline 150%](https://docs.pycom.io/img/lopy-pinout.png)

---

[Expansion Board Pinout](https://docs.pycom.io/chapter/datasheets/boards/expansion.html)
![inline 150%](https://docs.pycom.io/img/expansion-pinout.png)

---

## Prerequisites

- LoPy and Pymaker expansion board

- Install the Atom editor [https://atom.io/](https://atom.io/)

- Install the Pymakr Atom plugin [https://github.com/pycom/pymakr-atom](https://github.com/pycom/pymakr-atom)

- Update/install the latest FTDI drivers [http://www.ftdichip.com/Drivers/VCP.htm](http://www.ftdichip.com/Drivers/VCP.htm)

- Plug in the board to the USB and confirm the board is available:

```
% ls /dev/tty.usb*
/dev/tty.usbserial-DQ0090ZQ
```

---

## Update PyCom Firmware

### What you will need:
- Pycom Firmware Updater [https://pycom.io/downloads/](https://pycom.io/downloads/)
- Jumper Wire

---

## Update PyCom Firmware

### Steps:
- Install the jumper wire across `GND` (2nd pin from top on the right side) and `G23` (4th pin from the top on the left side)
- Plug in the board to the USB
- Run the Pycom Firmware Updater
- Remove the Jumper and reset the board

---

## Micropython Startup:

2 files are used at startup:

- `boot.py` - executed when the board boots up (if it exists)
- `main.py` - executed after the boot.py

If neither `boot.py` or `main.py` exist -or- the application exits, the board is put in the `REPL` mode.

---

## Connecting to LoPy:

- `username`: 'micro'
- `password`: 'python'

[Serial USB REPL](https://docs.pycom.io/chapter/toolsandfeatures/repl/serial.html):
- `screen /dev/tty.usbserial-* 115200`

[Telnet REPL](https://docs.pycom.io/chapter/toolsandfeatures/repl/telnet.html):
- Connect to default access point starting with: `lopy-`
- WiFi Password: www.pycom.io
- `telnet 192.168.4.1`

[FTP](https://docs.pycom.io/chapter/toolsandfeatures/FTP.html):
- URL: ftp://192.168.4.1
- `ftp 192.168.4.1`, then go into `passive` mode

---

## Pymakr Atom Plugin Configuration

- `pymakr.conf` - create this in the root of your project folder.  Set the `address` to the name of the USB Serial file name (e.g. `/dev/tty.usbserial-DQ0090ZQ`)

```javascript
{
    "address": "/dev/cu.usbserial-DQ0090ZQ",
    "username": "micro",
    "password": "python",
    "sync_folder": ""
}

```

---

[MicroPython-specific Libraries](http://docs.micropython.org/en/latest/wipy/library/index.html):

- `btree` – simple BTree database
- `framebuf` — Frame buffer manipulation
- `machine` — functions related to the hardware
- `micropython` – access and control MicroPython internals
- `network` — network configuration
- `uctypes` – access binary data in a structured way

---

###Building the firmeware locally and installing it (OSX)

1. Clone the `micropython-esp32` repo: [https://github.com/micropython/micropython-esp32](https://github.com/micropython/micropython-esp32)
1. Lookup the version of the ESP-IDF source needed.  E.g.
```/Users/carl/dev/github/carledwards/micropython-esp32/ports/esp32/Makefile:ESPIDF_SUPHASH := 2c95a77cf93781f296883d5dbafcdc18e4389656
```
1. Clone the `esp-idf` repo from espressif: [https://github.com/espressif/esp-idf](https://github.com/espressif/esp-idf)
1. Check out the version found in the `ESPIDF_SUPHASH` value from the step above.
1. Read the docs: [https://esp-idf.readthedocs.io/en/v2.0/macos-setup.html](https://esp-idf.readthedocs.io/en/v2.0/macos-setup.html)
1. Setup an alias to `python 2.7`:
```sudo ln -sf /usr/bin/python2.7 /usr/local/bin/python2
```
1. Set the ESPIDF home directory.  E.g.:
```ESPIDF=/Users/carl/dev/github/carledwards/esp-idf
```
1. build it
```make clean
make
```
1. The output is placed into the `build` directory.  E.g.:
```/Users/carl/dev/github/carledwards/micropython-esp32/ports/esp32/build
firmware.bin
```


---

#Projects

---

###Project 1: SSID Weather

![inline 70%](https://github.com/carledwards/ssid-weather/raw/master/images/project_1.png) ![inline 70%](https://github.com/carledwards/ssid-weather/raw/master/images/project_2.png)

######[https://github.com/carledwards/ssid-weather](https://github.com/carledwards/ssid-weather)

---

###Project 1: SSID Weather

![inline 40%](https://github.com/carledwards/ssid-weather/raw/master/images/iphone_wifi.png)

######[https://github.com/carledwards/ssid-weather](https://github.com/carledwards/ssid-weather)

---

###Project 1: SSID Weather

```python
from machine import Pin
from network import WLAN
from time import sleep
from DHT22RinusW import DHT22

dht_pin=Pin('P9', Pin.OPEN_DRAIN) # connect DHT22 sensor data line to pin P9/G16 on the expansion board
lastSSID = ""
while True:
    dht_pin(1) # drive pin high to initiate data conversion on DHT sensor
    tempInC, hum = DHT22(dht_pin)
    tempInF = round((tempInC/10) * 1.8 + 32)
    humidity = round(hum/10)
    SSID = "Temp: %d  Hum: %d" % (tempInF, humidity)
    if not lastSSID == SSID:
        wlan = WLAN(mode=WLAN.AP, ssid=SSID, auth=(WLAN.WPA2, '<YOUR PASSWORD HERE>'), channel=11, antenna=WLAN.EXT_ANT)
        print ("Changed SSID:", SSID)
        lastSSID = SSID
    sleep(60*5) # update every 5 minutes
```
######[https://github.com/carledwards/ssid-weather](https://github.com/carledwards/ssid-weather)

---
###Project 1: SSID Weather
####My Learnings
- Creating an access point is very easy:

```python
        wlan = WLAN(mode=WLAN.AP, ssid=SSID, auth=(WLAN.WPA2, '<YOUR PASSWORD HERE>'), channel=11, antenna=WLAN.EXT_ANT)
```
- Updated `DHT22` library to perform better sampling for interpreting the data returned

######[https://github.com/carledwards/ssid-weather](https://github.com/carledwards/ssid-weather)

---

###Project 2: Panel Meter Clock

![inline 20%](https://github.com/carledwards/panel-meter-clock/raw/master/pictures/clock_1.jpg)

######[https://github.com/carledwards/panel-meter-clock](https://github.com/carledwards/panel-meter-clock)

---

###Project 2: Panel Meter Clock

![inline 15%](https://github.com/carledwards/panel-meter-clock/raw/master/pictures/clock_4.jpg)

######[https://github.com/carledwards/panel-meter-clock](https://github.com/carledwards/panel-meter-clock)

---
###Project 2: Panel Meter Clock
####My Learnings
- Threading makes building Microcontroller apps a lot more manageable (i.e. not having processing loop)

```python
def update_clock_face_loop(delay):
    while App.currentState == State.CLOCK:
        update_current_time_from_rtc()
        update_clock_face(App.currentHour, App.currentMinute, App.currentSecond)
        time.sleep(delay)

_thread.start_new_thread(update_clock_face_loop, (.3,))

```

######[https://github.com/carledwards/panel-meter-clock](https://github.com/carledwards/panel-meter-clock)

---

###Project 2: Panel Meter Clock
####My Learnings
- `IRQ` callbacks for the Pin make handing external input detection easy:

```python
    def callback(p):
        val = p.value()
        print(val)

    pin = Pin('P13', Pin.IN)
    pin.callback(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=callback)

```
######[https://github.com/carledwards/panel-meter-clock](https://github.com/carledwards/panel-meter-clock)

---


###Project 2: Panel Meter Clock
####Implementation
- added very simple threaded web server:

```python
    WLAN(mode=WLAN.AP, ssid='Panel Meter Clock', auth=(None))
    ...
    webServer = WebServer(80, 'index.htm', webserver_request_callback, True)
    webServer.start()
    ...
    webServer.stop()
```

- added parameter options for callback:

```shell
    curl "http://192.168.4.1/ignored?hour=5&minute=30&second=20"
```

---

#Demos

---
##Demo 1: Set RGB LED to Green

```python
import pycom
pycom.heartbeat(False)
pycom.rgbled(0x00FF00)
```
<br>
<br>
<br>
######Source: [https://docs.pycom.io/chapter/tutorials/all/rgbled.html](https://docs.pycom.io/chapter/tutorials/all/rgbled.html)

---

##Slow Pulse RGB LED

```python
import pycom
import time

pycom.heartbeat(False)

while True:
  for color in range(256):
    pycom.rgbled(color << 16)
    time.sleep(.004)

  for color in range(255,0, -1):
    pycom.rgbled(color << 16)
    time.sleep(.004)
```

---

##Threading and IRQ callbacks

```python
import pycom
import time
from machine import Pin
import _thread

led = Pin('P9', Pin.OUT)
button = Pin('P10', Pin.IN, pull=Pin.PULL_UP)

def button_cb(p):
    print('button_cb: %d', p.value())

def blink_led():
    while(True):
        print('led ON')
        led.value(0)
        time.sleep(1)
        print('led OFF')
        led.value(1)
        time.sleep(1)

# Start LED Blink Thread
_thread.start_new_thread(blink_led, ())

# Set Button IRQ callback
button.callback(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=button_cb)

while True:
    print('button value: %d', button.value())
    time.sleep(2)
```

---

## End
