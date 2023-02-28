void wheel_move(byte dir)
{
    
    byte wheel_speed = 400;
    
    switch(dir)
    {   // Switch between priority directions
        case 0: // all stop 
          step_wheel(0,0);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Stop ");
          break; 
        case 1: // Forward sequence 1100 
          step_wheel(B10011111, B10010000);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Forward ");
          break; 
        case 2: // Backward sequence 0011
          step_wheel(B01101111, B01100000);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Backward ");
          break;
        case 3: //Left sequence 1001
          step_wheel(B11001111, B11000000);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Left ");
          break;
        case 4: //Right sequence 0110
          step_wheel(B00111111, B00110000);
          delayMicroseconds(wheel_speed); 
          //Serial.print(" Right ");
          break;
        case 5: //-rz sequence 0000
          step_wheel(B11111111, B11110000);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Clockwise ");
          break;    
        case 6: //+rz sequence 1111
          step_wheel(B00001111, B00000000);
          delayMicroseconds(wheel_speed);
          //Serial.print(" Anticlockwise ");
          break;     
    }
}


void step_wheel(byte step_high, byte step_low)
{
    /* 
    Bit Reference for step and direction pins to wheel drivers 
    const byte step0 = B00000001;
    const byte step1 = B00000010;
    const byte step2 = B00000100;
    const byte step3 = B00001000;
    const byte dir0  = B00010000;
    const byte dir1  = B00100000;
    const byte dir2  = B01000000;
    const byte dir3  = B10000000;
    */
    
    // Define shift register pins
    const byte data_pin  = 8;
    const byte latch_pin = 9;
    const byte clock_pin = 10;

    // Set pin modes 
    pinMode(latch_pin,OUTPUT);
    pinMode(data_pin, OUTPUT);
    pinMode(clock_pin,OUTPUT);

    /* 
      Set latch LOW to load data into register
      Send data to shift register with shiftOut 
      Set latch HIGH to shift data out of register 
     */
    // HIGH pulse 
    digitalWrite(latch_pin, LOW);
    shiftOut(data_pin, clock_pin, MSBFIRST, step_high);
    digitalWrite(latch_pin, HIGH);
    
    // Delay 
    delayMicroseconds(4);

    // LOW pulse 
    digitalWrite(latch_pin, LOW);
    shiftOut(data_pin, clock_pin, MSBFIRST, step_low);
    digitalWrite(latch_pin, HIGH);
}
