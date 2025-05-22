import i2c
import gpio
import lsm6ds3
import math

/**
FIFO test for LSM6DS3 sensor.
This example demonstrates:
- Configuring the FIFO to collect accelerometer and gyroscope data
- Setting a watermark threshold
- Reading data from the FIFO when the watermark is reached
*/

main:
  // Initialize I2C bus
  bus := i2c.Bus
    --sda=gpio.Pin 0
    --scl=gpio.Pin 1

  device := bus.device lsm6ds3.I2C_ADDRESS
  sensor := lsm6ds3.LSM6DS3 device

  // Enable sensor with default settings
  sensor.enable
    --dataRate_acc=lsm6ds3.ODR_XL_416Hz  // Higher data rate for FIFO demo
    --dataRate_gyro=lsm6ds3.ODR_G_416Hz  // Higher data rate for FIFO demo
    --range_acc=lsm6ds3.FS_XL_4G
    --range_gyro=lsm6ds3.FS_G_500DPS

  // Configure FIFO
  // Set watermark threshold to 10 samples
  // Use FIFO mode with no decimation and 104Hz FIFO data rate
  watermark_threshold := 10
  print_ "Configuring FIFO with watermark threshold: $watermark_threshold"
  
  sensor.fifoBegin
    --gyroFifoDecimation=0      // No decimation
    --accelFifoDecimation=0     // No decimation
    --threshold=watermark_threshold
    --mode=lsm6ds3.FIFO_MODE_FIFO
    --fifoDataRate=lsm6ds3.FIFO_ODR_104Hz

  // Wait for initial configuration to settle
  sleep --ms=100
  
  // Main loop
  iteration := 0
  while iteration < 5:  // Run for 5 iterations
    print_ "\n--- Iteration $iteration ---"
    
    // Check FIFO status
    sample_count := sensor.get_fifo_sample_count
    print_ "FIFO sample count: $sample_count"
    print_ "FIFO watermark reached: $(sensor.fifo_watermark_reached)"
    print_ "FIFO full: $(sensor.fifo_full)"
    
    // If watermark reached or FIFO full, read all samples
    if sensor.fifo_watermark_reached or sensor.fifo_full:
      print_ "Reading $sample_count samples from FIFO:"
      
      accelerometer_data := []
      gyroscope_data := []
      
      // Read all samples from FIFO
      // Note: In a real application, you would need to interpret the FIFO data format
      // This is a simplified example just showing raw values
      sample_count.repeat:
        sample := sensor.fifoSampleRead
        accelerometer_data.add sample
        sample := sensor.fifoSampleRead
        gyroscope_data.add sample
      
      // Print some of the collected data
      max_display := min 5 accelerometer_data.size
      print_ "First $max_display accelerometer samples:"
      max_display.repeat: |i|
        print_ "  Sample $i: $(accelerometer_data[i])"
      
      print_ "First $max_display gyroscope samples:"
      max_display.repeat: |i|
        print_ "  Sample $i: $(gyroscope_data[i])"
      
      iteration++
    
    // Also display current (non-FIFO) sensor readings for comparison
    if sensor.accelerationAvailable:
      acc := sensor.read_acceleration
      print_ "Current Accel: $(acc.x), $(acc.y), $(acc.z) m/s²"
    
    if sensor.gyroAvailable:
      gyro := sensor.read_gyro
      print_ "Current Gyro: $(gyro.x), $(gyro.y), $(gyro.z) dps"
    
    // Wait before checking FIFO again
    sleep --ms=500
  
  // After test completes, switch back to bypass mode to disable FIFO
  sensor.fifoBegin --mode=lsm6ds3.FIFO_MODE_BYPASS
  print_ "FIFO test complete" 