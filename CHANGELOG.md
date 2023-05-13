# 2023-05-13
* support for packing sensor data into 8-bit or 16-bit unsigned integers

  Is it possible now to scale and pack sensor value into 8-bit / 16-bit unsigned integer, to reduce size in TPDO frame. Sensor value is scaled to 8-bit or 16-bit width before transmission and scaled back on receive. Following additional parameters are recognized:
  * size (in bytes), valid values are 1 (8-bit int) or 2 (16-bit int)
  * min_value: min_value of sensor, used for scaling
  * max_value: max value of sensor, used for scaling
  values outside of range are clipped into [min_value, max_value]