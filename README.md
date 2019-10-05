# PyRPlidar ![PyPI - Python Version](https://img.shields.io/pypi/pyversions/pyrplidar) ![PyPI](https://img.shields.io/pypi/v/pyrplidar) [![MIT License](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/Hyun-je/pyrplidar/blob/master/LICENSE)

![61845532-a5413e80-aede-11e9-9eee-db8438055619](https://user-images.githubusercontent.com/7419790/61871806-b14bf100-af1c-11e9-94a6-812b4f10930a.png)

## Introduction
[PyRPlidar](https://github.com/Hyun-je/pyrplidar) is a python library for Slamtec RPLIDAR series.

* Supports all series (A1, A2 and A3)
* Implement all features of the device
* Simple code & Easy to use
* Use generator pattern (for performance)

![ezgif-5-3c5261a0b7fa](https://user-images.githubusercontent.com/7419790/66256236-b94ec980-e7c6-11e9-921e-c5098fce58b1.gif)


## Installation
```sh
$ pip install pyrplidar
```

## Example Code
```Python
from pyrplidar import PyRPlidar

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/cu.SLAB_USBtoUART"
# Windows : "COM5"


info = lidar.get_info()
print("info :", info)

health = lidar.get_health()
print("health :", health)

samplerate = lidar.get_samplerate()
print("samplerate :", samplerate)


scan_modes = lidar.get_scan_modes()
print("scan modes :")
for scan_mode in scan_modes:
    print(scan_mode)


lidar.disconnect()
```

## Documentation
This library implement full specifications on the [protocol documentation](http://bucket.download.slamtec.com/ccb3c2fc1e66bb00bd4370e208b670217c8b55fa/LR001_SLAMTEC_rplidar_protocol_v2.1_en.pdf) of Slamtec.
