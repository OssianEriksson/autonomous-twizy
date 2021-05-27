# piksi_multi_rtk <!-- omit in toc -->

ROS driver for the Swiftnav Pikis Multi RTK GNSS kit.

This package requires the python package sbp which can be installed with
```bash
pip3 install sbp
```

**Contents**

- [Nodes](#nodes)
  - [piksi_multi_rtk](#piksi_multi_rtk)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)

# Nodes

## piksi_multi_rtk

ROS driver for the Swiftnav Pikis Multi RTK GNSS kit.

### Published Topics

* `imu/raw` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)  
  Measurement values from the IMU present on the Piksi Multi board.

* `mag/raw` ([sensor_msgs/MagneticField](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html)  
  Measurement values from the magnetometer present on the Piksi Multi board.

* `gnss/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html)  
  Measurement values from from the Piksi Multi RTK GNSS.

### Parameters

Default parameter values are moslty inspired by data collected from the [Piksi manual](https://www.swiftnav.com/resource-files/Piksi%20Multi/v2.4.15/Manual/PiksiMulti-settings-v2.4.15.pdf), [IMU spec](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) or [magnetometer spec](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/).

* `~tcp_addr` (`string`, default: `"192.168.0.222"`)  
  TCP address to Piksi Multi board. This is the default address from the factory.

* `~tcp_port` (`int`, default: `55555`)  
  TCP port to Piksi Multi board. This is the default port from the factory.

* `~imu_acc_sens` (`double`, default: `0.0023975`)  
  IMU accelerometer sensitivity (converts between raw data and m/s²). The default value corresponds to `4096` counts per _g_.

* `~imu_acc_cov` (`list`, default: `[0.241081, 0.0, 0.0, 0.0, 0.241081, 0.0, 0.0, 0.0, 0.241081]`)  
  IMU accelerometer covariance matrix. List must be of length 9.

* `~imu_gyro_sens` (`double`, default: `6.651407e-5`)  
  IMU gyroscope sensitivity (converts between raw data and rad/s). The default value corresponds to `262.4` counts per degree per second.

* `~imu_gyro_offset` (`list`, default: `[0.0, 0.0, 0.0]`)  
  IMU gyroscope offset, added to measurement values after `~imu_gyro_sens` has been accounted for.

* `~imu_gyro_cov` (`list`, default: `[0.002742, 0.0, 0.0, 0.0, 0.002742, 0.0, 0.0, 0.0, 0.002742]`)  
  IMU gyroscope covariance matrix. List must be of length 9.

* `~imu_frame` (`string`, default: `"piksi_imu"`)  
  `frame_id` for values published on `imu/raw`.

* `~mag_sens` (`double`, default: `1e-6`)  
  Magnetometer sensitivity (converts between raw data and T). The default value corresponds to 1e-6 T per mT.

* `~mag_offset` (`list`, default: `[0.0, 0.0, 0.0]`)  
  Magnetometer offset, added to measurement values after `~mag_sens` has been accounted for.

* `~mag_frame` (`string`, default: Value of `~imu_frame`)  
  `frame_id` for values published on `mag/raw`.

* `~gnss_frame` (`string`, default: `"piksi_gnss"`)  
  `frame_id` for values published on `gnss/fix`.
