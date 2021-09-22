# ISM330DHCX 6DoF IMU
The STMicroelectronics ISM330DHCX is  6DoF gyroscope and acclerometer with a full-scale acceleration range of ±2/±4/±8/±16 g and a wideangular rate range of ±125/±250/±500/±1000/±2000/±4000 dps that enable its usagein a broad range of applications.

This i2c driver library is built on the [I2Cdev](https://github.com/jrowberg/i2cdevlib) library by Jeff Rowberg.

This library is still a work in progress. It's creation was spurred on by the need to be able to configure the IMU beyond what other available libraries allow. Not all functionalities have been fully implemented. 

The datasheet and related technical documents for the ISM330DHCX can be found [here](https://www.st.com/en/mems-and-sensors/ism330dhcx.html).