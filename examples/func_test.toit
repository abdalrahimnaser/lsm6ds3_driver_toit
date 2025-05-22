import i2c
import gpio
import lsm6ds3


import gpio


main:
  bus := i2c.Bus
    --sda=gpio.Pin 0
    --scl=gpio.Pin 1

  device := bus.device lsm6ds3.I2C_ADDRESS
  
  sensor := lsm6ds3.LSM6DS3 device

  sensor.enable --enable_free_fall=true
  sensor.set_freeFall_config 0x01 0x00

  sensor.calibrate_gyro

  while 1:
    if sensor.accelerationAvailable:
      acc := sensor.read_acceleration
      print_ "Accel: $(acc.x), $(acc.y), $(acc.z) m/s²"
    
    if sensor.gyroAvailable:
      gyro := sensor.read_gyro
      print_ "Gyro: $(gyro.x), $(gyro.y), $(gyro.z) dps"
    
    if sensor.temperatureAvailable:
      temp := sensor.read_temperature
      print_ "Temp: $(temp) °C"

    if sensor.signMotionDetected:
      print_ "Significant Motion Detected!"

    if sensor.freeFallDetected:
      print_ "Free Fall Detected!"

    sleep --ms=1000
  