/**
Driver for the LSM6DS3 accelerometer.

Datasheet: https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf

Author: Abdalrahim Naser, Lightbug
*/

import math
import serial.device as serial
import serial.registers as serial


I2C_ADDRESS     ::= 0x6B

// Accelerometer ODR settings
ODR_XL_POWER_DOWN       ::= 0x00  // [0000] 0000: Power-down
ODR_XL_1_6Hz            ::= 0xB0  // [1011] 0000: 1.6 Hz (low power only)
ODR_XL_12_5Hz           ::= 0x10  // [0001] 0000: 12.5 Hz (low power)
ODR_XL_26Hz             ::= 0x20  // [0010] 0000: 26 Hz (low power)
ODR_XL_52Hz             ::= 0x30  // [0011] 0000: 52 Hz (low power)
ODR_XL_104Hz            ::= 0x40  // [0100] 0000: 104 Hz (normal mode)
ODR_XL_208Hz            ::= 0x50  // [0101] 0000: 208 Hz (normal mode)
ODR_XL_416Hz            ::= 0x60  // [0110] 0000: 416 Hz (high performance)
ODR_XL_833Hz            ::= 0x70  // [0111] 0000: 833 Hz (high performance)
ODR_XL_1660Hz           ::= 0x80  // [1000] 0000: 1.66 kHz (high performance)
ODR_XL_3330Hz           ::= 0x90  // [1001] 0000: 3.33 kHz (high performance)
ODR_XL_6660Hz           ::= 0xA0  // [1010] 0000: 6.66 kHz (high performance)

// Gyroscope ODR settings
ODR_G_POWER_DOWN        ::= 0x00  // [0000] 0000: Power down
ODR_G_12_5Hz            ::= 0x10  // [0001] 0000: 12.5 Hz (low power)
ODR_G_26Hz              ::= 0x20  // [0010] 0000: 26 Hz (low power)
ODR_G_52Hz              ::= 0x30  // [0011] 0000: 52 Hz (low power)
ODR_G_104Hz             ::= 0x40  // [0100] 0000: 104 Hz (normal mode)
ODR_G_208Hz             ::= 0x50  // [0101] 0000: 208 Hz (normal mode)
ODR_G_416Hz             ::= 0x60  // [0110] 0000: 416 Hz (high performance)
ODR_G_833Hz             ::= 0x70  // [0111] 0000: 833 Hz (high performance)
ODR_G_1660Hz            ::= 0x80  // [1000] 0000: 1.66 kHz (high performance)
ODR_G_3330Hz            ::= 0x90  // [1001] 0000: 3.33 kHz (high performance)
ODR_G_6660Hz            ::= 0xA0  // [1010] 0000: 6.66 kHz (high performance)

// Gyroscope full-scale selections
FS_G_125DPS              ::= 0x02  // 0000 00[1]0:   125 dps
FS_G_245DPS              ::= 0x00  // 0000 [00]00: 245 dps
FS_G_500DPS              ::= 0x04  // 0000 [01]00: 500 dps
FS_G_1000DPS             ::= 0x08  // 0000 [10]00: 1000 dps
FS_G_2000DPS             ::= 0x0C  // 0000 [11]00: 2000 dps

// Accelerometer full-scale selections
FS_XL_2G                 ::= 0x00 // 0000 [00]00
FS_XL_4G                 ::= 0x08 // 0000 [10]00
FS_XL_8G                 ::= 0x0C // 0000 [11]00
FS_XL_16G                ::= 0x04 // 0000 [01]00


// Accelerometer output registers
OUTX_L_XL               ::= 0x28
OUTX_H_XL               ::= 0x29
OUTY_L_XL               ::= 0x2A
OUTY_H_XL               ::= 0x2B
OUTZ_L_XL               ::= 0x2C
OUTZ_H_XL               ::= 0x2D

// Gyroscope output registers
OUTX_L_G          ::= 0x22
OUTX_H_G           ::= 0x23
OUTY_L_G           ::= 0x24
OUTY_H_G           ::= 0x25
OUTZ_L_G          ::= 0x26
OUTZ_H_G           ::= 0x27

// Temperature output registers
OUT_TEMP_L          ::= 0x20
OUT_TEMP_H          ::= 0x21


// control registers
LPF1_BW_SEL              ::= 0x02 // 0000 00[1]0 (enable LPF1 of acc. @ cutoff = ODR/4)
LPF1_SEL_G               ::= 0x02 // 0000 00[1]0 (enable LPF1 of gyro)
FTYPE_G                  ::= 0x01 // 0000 00[01] (medium BW selection in LPF1-gyro)
XL_HM_MODE               ::= 0x10 // 000[1] 0000 (high-performance mode disabled for acc.)
G_HM_MODE                ::= 0x80 // [1]000 0000 (high-performance mode disabled for gyro)
WHO_AM_I         ::= 0x0F // this is the address where chip id is written
CTRL1_XL        ::= 0x10
CTRL2_G        ::= 0x11
CTRL3_C        ::= 0x12
CTRL4_C        ::= 0x13
CTRL5_C        ::= 0x14
CTRL6_C        ::= 0x15
CTRL7_G        ::= 0x16
CTRL8_XL        ::= 0x17
CTRL9_XL        ::= 0x18
CTRL10_C        ::= 0x19
SM_THS          ::= 0x13
FUNC_CFG_ACCESS ::= 0x01
WAKE_UP_SRC     ::= 0x1B
WAKE_UP_DUR     ::= 0x5C
STATUS_REG ::= 0x1E
FUNC_SRC1 ::= 0x53
FREE_FALL ::= 0x5D
TAP_CFG ::= 0x58
CHIP_ID          ::= 0x6A
MD1_CFG ::= 0x5E
GRAVITY_CONSTANT ::= 9.80665


class LSM6DS3:
  // The currently selected range.
  range_acc_/int := 0
  range_gyro_/int := 0

  // The currently selected ODR.
  dataRate_acc_/int := 0
  dataRate_gyro_/int := 0


  // Gyroscope calibration values
  gyro_offset_x/float := 0.0
  gyro_offset_y/float := 0.0
  gyro_offset_z/float := 0.0


  // device registers
  reg_/serial.Registers ::= ?
  
  
  constructor dev/serial.Device:
    sleep --ms=35 // Sensor turn-on time according to datasheet
    reg_ = dev.registers
    print_ "WHO_AM_I (current): $(reg_.read_u8 WHO_AM_I)"
    if ((reg_.read_u8 WHO_AM_I) != CHIP_ID):
      if ((reg_.read_u8 WHO_AM_I) != 0x69):
        throw "INVALID_CHIP/NONCONNECTED"

  enable -> none
      --dataRate_acc/int = ODR_XL_104Hz
      --dataRate_gyro/int = ODR_G_104Hz
      --range_acc/int = FS_XL_2G
      --range_gyro/int = FS_G_125DPS
      --enable_sig_motion/bool = false
      --enable_free_fall/bool = false:
    

    if not [ODR_XL_1_6Hz, ODR_XL_12_5Hz, ODR_XL_26Hz, ODR_XL_52Hz, ODR_XL_104Hz, ODR_XL_208Hz, ODR_XL_416Hz, ODR_XL_833Hz, ODR_XL_1660Hz, ODR_XL_3330Hz, ODR_XL_6660Hz].contains dataRate_acc:
      throw "INVALID_ODR_ACC"
    if not [ODR_G_12_5Hz, ODR_G_26Hz, ODR_G_52Hz, ODR_G_104Hz, ODR_G_208Hz, ODR_G_416Hz, ODR_G_833Hz, ODR_G_1660Hz, ODR_G_3330Hz, ODR_G_6660Hz].contains dataRate_gyro:
      throw "INVALID_ODR_GYRO"
    if not [FS_XL_2G, FS_XL_4G, FS_XL_8G, FS_XL_16G].contains range_acc:
      throw "INVALID_RANGE_ACC"
    if not [FS_G_125DPS, FS_G_245DPS, FS_G_500DPS, FS_G_1000DPS, FS_G_2000DPS].contains range_gyro:
      throw "INVALID_RANGE_GYRO"


    ctrl6 := XL_HM_MODE | FTYPE_G
    ctrl7 := G_HM_MODE
    ctrl1_xl := dataRate_acc | range_acc | LPF1_BW_SEL
    ctrl2_g := dataRate_gyro | range_gyro


    reg_.write_u8 CTRL4_C LPF1_SEL_G
    reg_.write_u8 CTRL6_C ctrl6
    reg_.write_u8 CTRL7_G ctrl7
    reg_.write_u8 CTRL1_XL ctrl1_xl
    reg_.write_u8 CTRL2_G ctrl2_g


    if enable_sig_motion:
      reg_.write_u8 CTRL10_C 0x05
    if enable_free_fall:
      reg_.write_u8 TAP_CFG 0x80
      
    // Route free-fall interrupt to INT1
    reg_.write_u8 MD1_CFG 0x10

    range_acc_ = range_acc
    range_gyro_ = range_gyro

    dataRate_acc_ = dataRate_acc
    dataRate_gyro_ = dataRate_gyro

  /**
  Reads the acceleration on the x, y and z axis.
  The returned values are in in m/s².
  */
  read_acceleration -> math.Point3f:
    x := reg_.read_i16_le (OUTX_L_XL) //autoincrementing is enabled by default in CTRL3_C 
    y := reg_.read_i16_le (OUTY_L_XL)
    z := reg_.read_i16_le (OUTZ_L_XL)

    // - RANGE_2G:  0.061 mg/LSB
    // - RANGE_4G:  0.122 mg/LSB
    // - RANGE_8G:  0.244 mg/LSB
    // - RANGE_16G: 0.488 mg/LSB 

    ACCEL_SENSITIVITIES ::= [0.061, 0.122, 0.244, 0.488]
    sensitivity := 0
    if range_acc_==FS_XL_2G:
      sensitivity=ACCEL_SENSITIVITIES[0]
    else if range_acc_==FS_XL_4G:
      sensitivity=ACCEL_SENSITIVITIES[1]
    else if range_acc_==FS_XL_8G:
      sensitivity=ACCEL_SENSITIVITIES[2]
    else if range_acc_==FS_XL_16G:
      sensitivity=ACCEL_SENSITIVITIES[3]

    // convert to m/s^2
    factor := sensitivity * GRAVITY_CONSTANT / 1000.0 

    return math.Point3f
        x * factor
        y * factor 
        z * factor


  /**
  Reads the gyroscope on the x, y and z axis.
  The returned values are in dps.
  */
  read_gyro_raw -> math.Point3f:
    x := reg_.read_i16_le (OUTX_L_G)
    y := reg_.read_i16_le (OUTY_L_G)
    z := reg_.read_i16_le (OUTZ_L_G)

    // Gyroscope sensitivity depends on the selected range
    // - FS_G_125DPS:  4.375 mdps/LSB
    // - FS_G_245DPS:  8.75 mdps/LSB
    // - FS_G_500DPS:  17.50 mdps/LSB
    // - FS_G_1000DPS: 35.00 mdps/LSB
    // - FS_G_2000DPS: 70.00 mdps/LSB
    
    GYRO_SENSITIVITIES ::= [4.375,8.75, 17.50, 35.0, 70.0]
    sensitivity := 0
    if range_gyro_==FS_G_125DPS:
      sensitivity=GYRO_SENSITIVITIES[0]
    else if range_gyro_==FS_G_245DPS:
      sensitivity=GYRO_SENSITIVITIES[1]
    else if range_gyro_==FS_G_500DPS:
      sensitivity=GYRO_SENSITIVITIES[2]
    else if range_gyro_==FS_G_1000DPS:
      sensitivity=GYRO_SENSITIVITIES[3]
    else:
      sensitivity=GYRO_SENSITIVITIES[4]

    // Convert to degrees per second
    factor := sensitivity / 1000.0

    return math.Point3f
        x * factor 
        y * factor 
        z * factor


  read_gyro -> math.Point3f:
    offset := math.Point3f gyro_offset_x gyro_offset_y gyro_offset_z
    return (read_gyro_raw - offset)

  calibrate_gyro -> none:
    gyro_offset_x  = read_gyro_raw.x
    gyro_offset_y  = read_gyro_raw.y
    gyro_offset_z  = read_gyro_raw.z
     
  /**
  Reads the temperature.
  The returned value is in degrees Celsius.
  */
  read_temperature -> float:
    temp := (reg_.read_i16_le OUT_TEMP_L) / 256.0 + 25.0
    return temp

  /**
  Sets the threshold for the sig_motion detection.
  The threshold is a value between 0 and 255.
  */
  set_sigMotion_config threshold/int -> none:
    reg_.write_u8 FUNC_CFG_ACCESS 0x80 // enable reg bank A 
    reg_.write_u8 SM_THS threshold
    reg_.write_u8 FUNC_CFG_ACCESS 0x00 // disable reg bank A


  /**
  Sets the threshold and duration for the free_fall detection.
  The threshold is a value between 0 and 7.
  The duration is a value between 0 and 63.
  Duration (in seconds) = duration[5:0] / ODR
  bit mapping for threshold: (000->156 mg, 001->219 mg, 010->250 mg, 011->312 mg, 100->344 mg, 101->406 mg, 110->469 mg, 111->500 mg)
  */
  set_freeFall_config threshold/int duration/int -> none:
    reg_.write_u8 FREE_FALL ((duration << 3) | threshold)
    reg_.write_u8 WAKE_UP_DUR (((duration & 0x20) << 2) | (reg_.read_u8 WAKE_UP_DUR)) // wake_up_dur[7]=duration[5]


  set_range range_acc/int=range_acc_ range_gyro/int=range_gyro_ -> none:
    reg_.write_u8 CTRL1_XL (((reg_.read_u8 CTRL1_XL) & 0b11110011) | range_acc)
    reg_.write_u8 CTRL2_G  (((reg_.read_u8 CTRL2_G) & 0b11110001) | range_gyro)
    range_acc_ = range_acc
    range_gyro_ = range_gyro

  set_dataRate dataRate_acc/int=dataRate_acc_ dataRate_gyro/int=dataRate_gyro_ -> none:
    reg_.write_u8 CTRL1_XL (((reg_.read_u8 CTRL1_XL) & 0xF0) | dataRate_acc)
    reg_.write_u8 CTRL2_G  (((reg_.read_u8 CTRL2_G) & 0xF0) | dataRate_gyro)
    dataRate_acc_ = dataRate_acc
    dataRate_gyro_ = dataRate_gyro


  accelerationAvailable -> bool:
    return ((reg_.read_u8 STATUS_REG) & 0x01) == 0x01

  gyroAvailable -> bool:
    return ((reg_.read_u8 STATUS_REG) & 0x02) == 0x02

  temperatureAvailable -> bool:
    return ((reg_.read_u8 STATUS_REG) & 0x04) == 0x04


  signMotionDetected -> bool:
    return ((reg_.read_u8 FUNC_SRC1) & 0x40) == 0x40

  freeFallDetected -> bool:
    return ((reg_.read_u8 WAKE_UP_SRC) & 0x20) == 0x20


