import i2c
import gpio
import lsm6ds3
import math

/**
Advanced FIFO test for LSM6DS3 sensor.
This example demonstrates:
- Configuring the FIFO in different modes
- Using FIFO for batch data collection
- Reading and parsing FIFO data according to the datasheet format
*/

// Function to display acceleration data in a readable format
display_acceleration data/math.Point3f:
  print_ "Acceleration: x=$(data.x.round 2), y=$(data.y.round 2), z=$(data.z.round 2) m/s²"

// Function to display gyroscope data in a readable format  
display_gyro data/math.Point3f:
  print_ "Gyroscope: x=$(data.x.round 2), y=$(data.y.round 2), z=$(data.z.round 2) dps"

main:
  // Initialize I2C bus
  bus := i2c.Bus
    --sda=gpio.Pin 0
    --scl=gpio.Pin 1

  device := bus.device lsm6ds3.I2C_ADDRESS
  sensor := lsm6ds3.LSM6DS3 device
  
  print_ "LSM6DS3 FIFO Advanced Test"
  
  // Enable sensor with specific settings for FIFO test
  sensor.enable
    --dataRate_acc=lsm6ds3.ODR_XL_416Hz
    --dataRate_gyro=lsm6ds3.ODR_G_416Hz
    --range_acc=lsm6ds3.FS_XL_4G
    --range_gyro=lsm6ds3.FS_G_500DPS

  // Run calibration to eliminate gyro bias
  print_ "Calibrating gyroscope, keep device still..."
  sensor.calibrate_gyro
  print_ "Calibration complete"
  
  // First, demonstrate FIFO in BYPASS mode (FIFO disabled)
  print_ "\n--- Test 1: FIFO in BYPASS mode ---"
  sensor.fifoBegin --mode=lsm6ds3.FIFO_MODE_BYPASS
  print_ "FIFO set to BYPASS mode. FIFO is effectively disabled."
  print_ "Reading direct sensor data:"
  
  3.repeat:
    if sensor.accelerationAvailable:
      acc := sensor.read_acceleration
      display_acceleration acc
      
    if sensor.gyroAvailable:
      gyro := sensor.read_gyro
      display_gyro gyro
      
    sleep --ms=100
  
  // Second, demonstrate FIFO in FIFO mode with watermark
  print_ "\n--- Test 2: FIFO mode with watermark ---"
  watermark_threshold := 20  // Set watermark at 20 samples
  
  sensor.fifoBegin
    --gyroFifoDecimation=0
    --accelFifoDecimation=0
    --threshold=watermark_threshold
    --mode=lsm6ds3.FIFO_MODE_FIFO
    --fifoDataRate=lsm6ds3.FIFO_ODR_104Hz
  
  print_ "FIFO configured in FIFO mode with watermark at $watermark_threshold samples"
  print_ "Waiting for FIFO to fill to watermark..."
  
  watermark_reached := false
  timeout := 30  // 3 seconds timeout
  
  while not watermark_reached and timeout > 0:
    watermark_reached = sensor.fifo_watermark_reached
    if not watermark_reached:
      print_ "FIFO sample count: $(sensor.get_fifo_sample_count)"
      sleep --ms=100
      timeout--
  
  if watermark_reached:
    print_ "Watermark reached! Reading FIFO data..."
    sample_count := sensor.get_fifo_sample_count
    print_ "Samples in FIFO: $sample_count"
    
    // Read and process data from FIFO
    // Note: In the LSM6DS3, the FIFO can be configured to store data from 
    // accelerometer and gyroscope in various patterns based on decimation factors
    
    // For this simple example, we assume alternating accelerometer and gyroscope data
    // In a real application, you would need to handle the pattern based on your FIFO configuration
    
    5.repeat: |i|
      if sample_count >= 2:  // Need at least 2 samples (1 accel + 1 gyro)
        // For demonstration, we'll print raw values
        // In an actual application, these would need proper scaling based on selected ranges
        accel_raw := sensor.fifoSampleRead
        gyro_raw := sensor.fifoSampleRead
        print_ "Sample pair $i: Accel=$accel_raw, Gyro=$gyro_raw"
        sample_count -= 2
  else:
    print_ "Timeout waiting for watermark. FIFO sample count: $(sensor.get_fifo_sample_count)"
  
  // Third, demonstrate FIFO in CONTINUOUS (STREAM) mode
  print_ "\n--- Test 3: FIFO in CONTINUOUS (STREAM) mode ---"
  
  sensor.fifoBegin
    --gyroFifoDecimation=0
    --accelFifoDecimation=0
    --threshold=0  // No watermark in this test
    --mode=lsm6ds3.FIFO_MODE_STREAM
    --fifoDataRate=lsm6ds3.FIFO_ODR_104Hz
  
  print_ "FIFO set to STREAM mode. Oldest data will be overwritten when FIFO is full."
  print_ "Collecting data for 3 seconds..."
  
  sleep --ms=3000  // Allow FIFO to collect data
  
  sample_count := sensor.get_fifo_sample_count
  print_ "Samples in FIFO after 3 seconds: $sample_count"
  
  // Read a maximum of 10 samples or whatever is available
  samples_to_read := min 10 sample_count
  if samples_to_read > 0:
    print_ "Reading $samples_to_read samples:"
    
    samples_to_read.repeat: |i|
      sample := sensor.fifoSampleRead
      print_ "Sample $i: $sample"
  else:
    print_ "No samples available in FIFO"
  
  // Finally, demonstrate STF (Stream-to-FIFO) mode - transitions from STREAM to FIFO on trigger
  print_ "\n--- Test 4: STF (Stream-to-FIFO) mode ---"
  print_ "STF mode transitions from STREAM to FIFO mode upon trigger"
  print_ "This is useful for capturing pre-trigger and post-trigger data"
  print_ "Note: A full implementation would require interrupt configuration"
  
  sensor.fifoBegin
    --gyroFifoDecimation=0
    --accelFifoDecimation=0
    --threshold=10
    --mode=lsm6ds3.FIFO_MODE_STF
    --fifoDataRate=lsm6ds3.FIFO_ODR_104Hz
  
  print_ "STF mode configured. For a complete implementation, an interrupt would be configured."
  print_ "In this simple example, we'll just simulate the behavior."
  print_ "Collecting data..."
  
  sleep --ms=2000
  
  sample_count := sensor.get_fifo_sample_count
  print_ "Samples in FIFO: $sample_count"
  
  // Reset FIFO to BYPASS mode when done
  sensor.fifoBegin --mode=lsm6ds3.FIFO_MODE_BYPASS
  print_ "FIFO tests complete" 