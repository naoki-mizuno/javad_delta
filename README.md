# JAVAD DELTA

Driver for JAVAD DELTA.

## Dependencies

- [serial](http://wiki.ros.org/serial)
- [nmea_msgs](http://wiki.ros.org/nmea_msgs)
- [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver)
- [gdal](https://www.gdal.org/)

GDAL can be installed on most systems with the package manager. On Ubuntu,
install it with:

```
$ sudo apt-get install python-gdal
```

You also need Internet access to retrieve the correction data from the ntrip
caster (`ntrip.jenoba.jp` in the example config).

## Configuration

You need to set the information about your ntrip caster. The configurations
that we use are written in the example. You need to set the `ntrip_user` and
`ntrip_password`, which should be provided by the caster.


## Thanks

[rosserial_python](https://github.com/ros-drivers/rosserial) for how to make a
Python ROS node. [ntrip_ros](https://github.com/tilk/ntrip_ros) for how to
communicate with the NTRIP caster.

## License

MIT

## Author

Naoki Mizuno
