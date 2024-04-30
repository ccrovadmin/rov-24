# Code for Coral Crusaders 2024 ROV

## UPLOADING TO ARDUINO FROM RASPBERRY PI
Instal the Arduino CLI on RPi with `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh` & add ~/bin to `PATH`.

### IF YOU ARE RUNNING A METRO (or other adafruit SAMD board)
Run `arduino-cli config init` and add `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` to your `arduino-cli.yaml` file in the  `board_manager: additional_urls:` list field. Then run `arduino-cli core update-index` & `arduino-cli core install adafruit:samd`.


Install the necessary libraries:
`arduino-cli lib install 'BlueRobotics MS5837 Library'`

And finally to upload the control code (from `arduino-slave/`):
`arduino-cli compile --fbqn [FBQN] .`
If you're running a Metro M4, your FBQN will be `adafruit:samd:adafruit_metro_m4`.


## Arduino Controller
Install the `MS5837` library to the Arduino IDE's Library Manager. Upload `arduino-slave/arduino-slave-ctl.ino` to your arduino controller. Edit pin constants, thruster directions as needed.

## Computer Topside
`cd computer-topside` & `pip install -r requirements.txt`. Run `python computer-topside-ctl.py` with a Logitech F310 gamepad plugged in to start topside server. Edit `HOST` IP to LAN IP of topside computer.

## Raspberry Pi 
Run `pip install -r requirements.txt`. `cd` into `rpi-master/nmea_encode_c_ext`, and run `python setup.py build_ext --inplace` & `cd ..`. Run `python rpi-master-ctl.py` with Arduino controller plugged into RPi.  Edit `HOST` ip to LAN IP of topside computer.
