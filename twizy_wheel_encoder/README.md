# twizy_wheel_encoder <!-- omit in toc -->

ROS drivers for custom wheel encoders on an autonomous Renault Twizy at the Department of Electrical Engineering, Chalmers University of Technology, Sweden.

**Package Links**

* [Msg/Srv API](https://ossianeriksson.github.io/autonomous-twizy/twizy_wheel_encoder/html/index-msg.html)

**Contents**

- [Installation](#installation)
- [Launching](#launching)
  - [Published Topics](#published-topics)

# Installation

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software/) and add yourself to the `dialout` group as per the installation instructions.
   
2. [Set up rosserial for Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup), if you face problems later try installing version `0.7.9` from the Arduino IDE directly according to [this answer](https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/?answer=362407#post-id-362407). This version may also require manual patching accoring to [this answer](https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/?answer=362427#post-id-362427). (The route of using version `0.7.9` and changing `<cstring>` to `<string.h>` is the only way I got it to work...)

3. [Generate message headers](http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages) for the [twizy_wheel_encoder/WheelEncoder](msg/WheelEncoder.msg) message type. I can't remember if there was trouble with this step also, but if there was I found the solution on google (possibly something about using `rosserial_arduino` instead of `rosserial_client` to generate the headers???).

4. Open [firmware.ino](firmware/firmware.ino) in the Arduino IDE, make sure the Arduino is plugged in to the computer and that the correct port is selected in the IDE. Then compile and upload the firmware to the Arduino from the IDE.

# Launching

Rosserial arduino works by launching a proxy node which communicates with the Arduino board through a serial port, publishing and subscribing to topics and reading parameters in it's place. A launch file is therefore used to start this proxy node and set relevant parameters: [wheel_encoders.launch](launch/wheel_encoders.launch).

## Published Topics

* `wheel/{left|right}/encoder` ([twizy_wheel_encoder/WheelEncoder](msg/WheelEncoder.msg))
  Wheel encoder speed measurements. These speeds are achieved by taking first order approximations of the wheel axis angle time derivative.