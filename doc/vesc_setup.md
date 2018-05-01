# VESC Setup

## Loading the firmware
To use the VESC to control the servo motor, it must be loaded with the right firmware. Download the firmware from [here](https://github.com/vedderb/bldc-tool/tree/master/firmwares). Choose the right version that corresponds to your VESC hardware! Uploading the wrong firmware version may damage the VESC. If purchased from Vanda Electronics, the version should be 4.12.  
To upload the firmware to the VESC, use the BLDC Tool.
```bash
$ sudo apt-get install qtcreator qt-sdk libudev-dev libqt5serialport5-dev
$ git clone https://github.com/vedderb/bldc-tool
$ cd bldc-tool
$ qmake -qt=qt5
$ make clean && make
```
Allow for serial access without using sudo:
```bash
$ sudo adduser $USER dialout
```
Restart for access changes to take effect
```bash
sudo reboot now
```
Start BLDC-tool from inside of the built repo
```bash
$ ./BLDC_Tool
```
Select the VESC from the serial connection (VESC must be powered for it to be detected) and connect. Go to the firmware tab, select the file  `VESC_servoout.bin` and upload.

### Loading the firmware without a bootloader
If the VESC doesn't come with a bootloader, uploading through USB won't work. An ST-Linkv2 device is needed.
First add udev rules to access the programmer without being root
```bash
$ wget vedder.se/Temp/49-stlinkv2.rules
$ sudo mv 49-stlinkv2.rules /etc/udev/rules.d/
$ sudo reload udev
```
Connect the ST-Linkv2 and it should appear in /dev as stlinkv2. Connect the ST-Link to the VESC with JST cables as shown in [this picture](http://vedder.se/wp-content/uploads/2014/12/EbayStlink_small.jpg) (or refer to the bottom side of the VESC).
```bash
$ git clone https://github.com/vedderb/bldc.git bldc-firmware
$ cd bldc-firmware
$ git checkout ed61c4d
```
Edit `conf_general.h` and select the right hardware version (default 410 is correct for VESC 4.12). Also change the macro `SERVO_OUT_ENABLE` to 1. Also download GCC ARM Embedded to compile the firmware.
```bash
$ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
$ sudo apt-get update
$ sudo apt-get install gcc-arm-embedded
$ make upload
```

## Configuring the VESC
Now with the BLDC Tool, configure the VESC. Make sure the sensors are connected and follow the instructions from the [website](http://vedder.se/2015/01/vesc-open-source-esc/).
1. Connect the motor without any load and make sure that it can spin up freely.
2. Make sure that no other input such as PPM is used. If it is, it will stop the motor immediately when the detection tries to start it and the detection will fail.
3. Click the “Start detection” button. The motor should spin up, release throttle and then run slowly for a moment.  
If the motor doesn’t spin up properly, Adjust “Current” and “Min ERPM” until it does. In general, small motors should have lower current and higher ERPM and larger motors the other way around. Current usually is in the range 1A to 6A and min ERPM usually is in the range 300 to 1200.  
If spinning up works but running slowly afterwards doesn’t (the motor just stutters), try increasing “Low duty” to 0.1 or so. Increasing low duty will make it easier for the motor to run slowly during the test, but the result will become less accurate.  
Manually put the obtained values into the boxes. I usually round “integrator limit” down to the closest multiple of 5 and “BEMF Coupling” down to the closest multiple of 50. Having them slightly lower than the detection result is good in most cases, so that’s why I round them downwards like that. Getting these parameters perfectly right is not too critical though.

Note that if running the motor at slow speed doesn't turn the motor but instead makes a high-pitched sound, minimum RPM should be increased.