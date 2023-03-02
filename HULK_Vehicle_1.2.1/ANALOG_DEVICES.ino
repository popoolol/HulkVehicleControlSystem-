// ============================= ACCELEROMETER =================================
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


// ============================= JOYSTICKS =================================

// Set a zero for the joysticks
// When the joysticks are turned on they do not always read the same zero, this will correct that 
void setup_joysticks()
{    
    // Reading joysticks once seems to be enough to get good shift values 
    read_joysticks();           
    joy0_x_shift = joy0_x_input;
    joy0_y_shift = joy0_y_input;
    joy1_x_shift = joy1_x_input;
    joy1_y_shift = joy1_y_input;   
}


// Gets joystick inputs: analog values in x and y and the buttons
// The buttons are toggled. joystick buttons change between controlling movement and controlling stability 
// pressing both buttons at once will set into autoleveling mode 
void read_joysticks()
{
    // Define joysitck pins
    const byte jx_0_pin   = A0;  // Analog input pins(joysticks) 
    const byte jy_0_pin   = A1;
    const byte jx_1_pin   = A2;
    const byte jy_1_pin   = A3;

    const int max_j = 511;
    
    // Begin by polling the joysticks for input
    joy0_x_input = map(analogRead(jx_0_pin), 0, 1024, max_j, -max_j) - joy0_x_shift; // Read x direction, remap to have negative values
    joy0_y_input = map(analogRead(jy_0_pin), 0, 1024, max_j, -max_j) - joy0_y_shift; // Read y direction, remap to have nagative values 
    joy1_x_input = map(analogRead(jx_1_pin), 0, 1024, max_j, -max_j) - joy1_x_shift;                
    joy1_y_input = map(analogRead(jy_1_pin), 0, 1024, max_j, -max_j) - joy1_y_shift; 
    
}


// ============================= BUZZER =================================
