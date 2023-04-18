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

}

// ============================= JOYSTICKS =================================

// Set a zero for the joysticks
// When the joysticks are turned on they do not always read the same zero, this will correct that 
void setup_joysticks()
{    
    joy0_x_shift = map(analogRead(A0), 0, 1024, 511, -511); // Read x direction, remap to have negative values
    joy0_y_shift = map(analogRead(A1), 0, 1024, 511, -511); // Read y direction, remap to have nagative values 
    joy1_x_shift = map(analogRead(A2), 0, 1024, 511, -511);                
    joy1_y_shift = map(analogRead(A3), 0, 1024, 511, -511); 
}
