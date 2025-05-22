/**
Test script for the LSM6DS3 FIFO features.
This example demonstrates:
- FIFO initialization and configuration
- Reading data from FIFO
- Checking FIFO status (watermark, full, sample count)
- Interpreting FIFO data as accelerometer and gyroscope values
*/

import gpio
import i2c
import lsm6ds3
import math

main:
  // Initialize I2C 
  bus := i2c.Bus
    --sda=gpio.Pin 0
    --scl=gpio.Pin 1
  
  device := bus.device lsm6ds3.I2C_ADDRESS
  sensor := lsm6ds3.LSM6DS3 device
  
  print "LSM6DS3 FIFO Test"
  
  // Basic sensor configuration
  sensor.enable
  
  // Configure FIFO
  // Set 32 as FIFO threshold
  threshold := 32
  
  // Initialize FIFO with:
  // - Accelerometer decimation factor of 1 (every sample)
  // - Gyroscope decimation factor of 1 (every sample)
  // - STREAM mode (continuously updates FIFO)
  // - FIFO data rate of 416Hz
  print "Configuring FIFO..."
  sensor.fifoBegin
    --accelFifoDecimation=1  // Use decimation factor of 1 (not 0)
    --gyroFifoDecimation=1   // Use decimation factor of 1 (not 0)
    --threshold=threshold
    --mode=lsm6ds3.FIFO_MODE_STREAM  // Use STREAM mode instead of FIFO mode
    --fifoDataRate=lsm6ds3.FIFO_ODR_416Hz
  
  // Configure INT1 pin to assert when FIFO threshold is reached
  sensor.int1_config
    --fifo_thresholdReached=true
  
  print "FIFO configured, waiting for data collection..."
  print "Threshold set to: $threshold"
  
  // Wait longer for FIFO to collect data (3 seconds should be plenty)
  print "Waiting 3 seconds for data collection..."
  sleep --ms=3000
  
  // Read and display FIFO status
  sample_count := sensor.get_fifo_sample_count
  print "FIFO sample count: $sample_count"
  print "FIFO threshold reached: $(sensor.fifo_watermark_reached)"
  print "FIFO full: $(sensor.fifo_full)"
  
  // Read all samples from FIFO
  if sample_count > 0:
    print "Reading FIFO samples:"
    
    // Calculate the number of complete data sets (1 set = 6 values: GX, GY, GZ, AX, AY, AZ)
    // Each FIFO slot holds one 16-bit value
    num_complete_sets := sample_count / 6
    
    // Define scale factors based on selected ranges
    // Gyroscope sensitivity for FS_G_500DPS is 17.50 mdps/LSB
    gyro_scale := 17.50 / 1000.0  // Convert to dps
    
    // Accelerometer sensitivity for FS_XL_4G is 0.122 mg/LSB
    accel_scale := 0.122 * lsm6ds3.GRAVITY_CONSTANT / 1000.0  // Convert to m/s²
    
    // Read up to 5 complete sets or whatever is available
    sets_to_read := min num_complete_sets 5
    
    print "Reading $sets_to_read complete data sets (gyro x,y,z + accel x,y,z):"
    
    for i := 0; i < sets_to_read; i++:
      print "Data set #$i:"
      
      // Read gyroscope data (X, Y, Z)
      gyro_x := sensor.fifoSampleRead * gyro_scale
      gyro_y := sensor.fifoSampleRead * gyro_scale
      gyro_z := sensor.fifoSampleRead * gyro_scale
      
      // Read accelerometer data (X, Y, Z)
      accel_x := sensor.fifoSampleRead * accel_scale
      accel_y := sensor.fifoSampleRead * accel_scale
      accel_z := sensor.fifoSampleRead * accel_scale
      
      // Display the data
      print "  Gyroscope (dps): X=$gyro_x, Y=$gyro_y, Z=$gyro_z"
      print "  Accelerometer (m/s²): X=$accel_x, Y=$accel_y, Z=$accel_z"
      print ""
  else:
    print "No samples collected. Debugging FIFO configuration..."
    // Try reading some regular data to verify the sensor is working
    print "Regular sensor readings:"
    accel := sensor.read_acceleration
    gyro := sensor.read_gyro
    print "  Accelerometer: $accel"
    print "  Gyroscope: $gyro"
  
  // Clear FIFO by switching to bypass mode and back
  print "Clearing FIFO..."
  sensor.fifoBegin
    --mode=lsm6ds3.FIFO_MODE_BYPASS
  
  print "FIFO test completed"

/**
Helper function to find minimum of two integers.
*/
min a/int b/int -> int:
  return a < b ? a : b 