# state-estimators

## Resources

Information which is of use for electrical design is in the [electrical-resources.md](Documents/electrical-resources.md) file.

Other resources:

* [IMU quadrotor dataset](http://www.sfly.org/mav-datasets)
* [A Collection of Outdoor Robotic Datasets with centimeter-accuracy Ground Truth](http://www.mrpt.org/malaga_dataset_2009)
* [Handheld camera 6dof trajectories](http://webdav.is.mpg.de/pixel/benchmark4camerashake/)

Papers:

* [Stochastic modeling of MEMS sensors](http://www.cit.iit.bas.bg/cit_2010/v10-2/31-40.pdf)
* [Attitude Estimation Using Modified Rodrigues Parameters](http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19960035754.pdf)
* [Analog Device's Application Note on the EKF implementation in another of their sensors](http://www.analog.com/media/en/technical-documentation/application-notes/AN-1157.pdf)
* [A Survey of Nonlinear Attitude Estimation Methods](http://ancs.eng.buffalo.edu/pdf/ancs_papers/2007/att_survey07.pdf)

Existing projects:

* https://github.com/sfwa/ukf
* [An existing sensor](https://www.bosch-sensortec.com/en/homepage/products_3/sensor_hubs/iot_solutions/bno055_1/bno055_4) which does sensor fusion whose SPI interface we may take inspiration from.
* https://github.com/kriswiner/BNO-055 - A driver for the above which does
    attitude estimation on an Arduino via a quaternion filter

