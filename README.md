
# CHICKEN RUN~

## Dependencies
### Web
- NodeJS
- NPM
```
# [replaced next line if encountered error on ubuntu 18.04] sudo apt-get install curl python-software-properties
sudo apt-get install -y curl software-properties-common
# Check the version you want to install, current LTS is 10.16.0
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs

# upgrade npm
sudo npm install -g npm@latest
```
- Nginx: install and create config file
```
sudo apt-get update
sudo apt-get install nginx -y
```

### ROS (Use your own version)
- [ceres-solver](http://ceres-solver.org/installation.html#linux)
- ros-**&lt;version&gt;**-usb-cam
- ros-**&lt;version&gt;**-rosbridge-suite
- ros-**&lt;version&gt;**-web-video-server

```
# change ROS version
sudo apt-get install ros-melodic-rosbridge-suite ros-melodic-web-video-server ros-melodic-usb-cam -y
```

### Python
- SMBus2
```
# install smbus for motor control
curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
sudo python get-pip.py
sudo pip install smbus2
```
---
## Before Start
### Website
- goto **src/chicken/server**
- run `npm install && ./modify_lib.sh`
### ROS
- I2C
```
sudo adduser <user> i2c
```
- SPI: Check slack for detail
- Make sure a SD card is mounted at ~/data for rosbag
```
# sudo mount <path-of-sdcard> <dir-wanna-mount>
sudo mount /dev/mmcblk2 /home/<user>/data/
# mount at startup
sudo vim /etc/fstab
```
---
## TODO
- Convert odometer to pose
- Pose estimated by UWB trilateration & odometry hasn't fused yet, same as the data visualization on rviz.
- Plot rivz to website. Now can control robot by any mobile device in the same network w/ robot, just trying to visualize all the data on the web.
- Need a new mechanical design that the current one can't support the weight well, or change a poweful motor. Ask Andy for the file.
- Add ammonia sensor to detect the value and plot on the map
- Require a charging docker. The charging unit and the battery monitor has been purchased. Use relay to switch run/charge states.
- The max working range of DWM1000 (~35m) doesn't match the usual range measured from others (>60m), maybe [use another product](https://github.com/thotro/arduino-dw1000/issues/129#issuecomment-482418845) or [enhance the TW_POWER](https://github.com/thotro/arduino-dw1000/issues/245)?
---
## Detail Informaiton is mentioned in [slack](https://robot304.slack.com).