// Begins i2c protocol 
// also determines an initial offset for the accelerometer. 
// If sitting on wheels on floor, this is assumed level 
// Comment out or don't use shift values if true level is desired
void setup_accel()
{
    // Begin accelerometer communication
    Wire.begin();                 // Begin accelerometer i2c connection
    Wire.beginTransmission(0x68); // I2C address of MPU 6050             
    Wire.write(B111);             // Reset accelerometer. Also initializes serial interface
    Wire.endTransmission(true);   // end communication
    
    Wire.beginTransmission(0x68); // I2C address of MPU 6050
    Wire.write(0x6B);             // Writing to clock configuration register
    Wire.write(0);                // Byte 0 selects internal 8MHz clock, also wakes up device
    Wire.endTransmission(true);   // end communication
    
    Wire.beginTransmission(0x68); 
    Wire.write(0x1C);             // Writing to acceleromter congfiguration register
    Wire.write(B11000);           // Setting the sensitivity to to lowest
    Wire.endTransmission(true);   // end communication
    
    Serial.print("Completed Accel Setup");
    Serial.println();
}

void read_accel()
{
    int16_t accel_x_raw;     // Creating 16 bit values for reading accelerometer data
    int16_t accel_y_raw;
    
    Wire.beginTransmission(0x68);   // Establish path for accelerometer data
    Wire.write(0x3B);               // Selecting register storing accelerometer data
    Wire.endTransmission(false);    // Continue transmission
    Wire.requestFrom(0x68,4,true);  // Retrieving 4 bytes from established path, true means stop retrieving 
  
    // Accelerometer x and y data are 16 bit values starting with most significant digit. X data is stored first 
    // 4 bytes are requested from the Gy 512, the first 2 bytes are x data and the second 2 bytes are y data
    // read function reads 8 bits of data at a time. 8 bits are read, those bits are shifted 8 places, then the next 8 bits are read
    accel_x_raw = Wire.read()<<8|Wire.read(); // read byte, shift byte 8 places, read second byte. 2 bytes of x data   
    accel_y_raw = Wire.read()<<8|Wire.read(); // 2 bytes of y data
    
    accel_y_input = accel_y_raw - accel_y_shift; // apply optional shift, shift starts at zero
    accel_x_input = accel_x_raw - accel_x_shift;
}


// Creates running average to smooth accelerometer values
void accel_smoother()
{  
    // generate array index position
    int accel_next_index = loop_counter % accel_average_count;
    
    // place accelerometer x and y values into array cells
    accel_x_running_average[accel_next_index] = accel_x_input;
    accel_y_running_average[accel_next_index] = accel_y_input;

    // Reset smoothed value
    int accel_x_smooth = 0;
    int accel_y_smooth = 0;

    // Sum all array elements 
    for(int i=0; i< accel_average_count; ++i)
    {
      accel_x_smooth += accel_x_running_average[i];
      accel_y_smooth += accel_y_running_average[i];
    }

    // Get running average by dividing array sum by amount of array elements 
    accel_x_input = accel_x_smooth / accel_average_count;
    accel_y_input = accel_y_smooth / accel_average_count;
}

// Create a relative zero for accelerometer. Generates shift values
void zero_accel()
{
    // Set shift to zero, incase a shift was previously applied 
    accel_x_shift = 0;
    accel_y_shift = 0;
    
    // loop over array elements of accelerometer running average
    for (byte shift_count = 0; shift_count < accel_average_count; shift_count++)
    {
          read_accel();     // Read accelerometer values
          accel_smoother(); // Smooth values 
          loop_counter++;   // Increment loop counter for array indexing 
    }
    
    // Use smoothed values from running average to set shift values
    accel_x_shift = accel_x_input;
    accel_y_shift = accel_y_input;
    
    Serial.print("Zero Accel Values");
    Serial.println();
    Serial.print("AccShift: ");Serial.print(accel_x_shift);Serial.print(" ");Serial.print(accel_y_shift);Serial.println();
}
 
