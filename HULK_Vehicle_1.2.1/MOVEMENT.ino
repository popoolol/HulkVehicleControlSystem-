// Move Stabilizer handles all stabilizer movement, manual or automatic
// * UART and Step-direction 
// * Controlling all at once, controlling individual 
// Broken into two main parts, uart and stepdir
// Both parts have the same number of cases, and the cases are the same for the two protocols 
// The vactual speed, or microstep_per_second is set in the function that calls move_stabilizer
byte stabilizer_move(byte dir){       

    // Default stallGaurd value, when not asked to pull it 
    //byte sg_raw = 255; 

    //uint32_t vactual_speed = max(stab_driver0.VACTUAL(),max(stab_driver1.VACTUAL(),stab_driver2.VACTUAL()));
    uint32_t vactual_speed = 80000;
    
    // The stabilizer movement speed is constant. There is no real need to have a variable speed
    
    bool shaft_direction = false;           // true turns motors forward, false turns motors backward. 
        
    switch(dir)// Switch between priority directions
    {   
        // The cases set the motor direction with shaft(), then sets the speed with VACTUAL()
        // Cases 0 - 2 mave all stabilizers simulataneously 
        // Cases 3 - 8 move stabilizers individually 
        // Cases 9 -10 move stabilizers 0 and 1 together (for leveling)
        case 0: // all stop 
        
            vactual_speed = 0;
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            //Serial.print(" ALL STOP ");
            break; 
            
        case 1: // All Up
            
            stab_driver0.shaft(!shaft_direction); // shaft sets motor direction
            stab_driver1.shaft(!shaft_direction); // !shaft_direction reverses direction
            stab_driver2.shaft(!shaft_direction); 
               
            stab_driver0.VACTUAL(vactual_speed);  // Set speed 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            //Serial.print(" All Up ");
            break;
            
        case 2: // All down
        
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);  
            stab_driver2.shaft(shaft_direction);     
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            //Serial.print(" All Down ");
            break;    
              
        case 3: // move stabilizer 0 up
            // Cases 3 - 8 also retrive the result of the stall gaurd for sensorless homing 
            stab_driver0.shaft(!shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);  // SG_RESULT() gets the stallgaurd value
            sg_raw = stab_driver0.SG_RESULT(); // This value is called IMMEDIATELY after movement for best results 
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0); 
            //Serial.print(" S0 Up ");
            break; 
        case 4: // move stabilizer 0 down
        
            stab_driver0.shaft(shaft_direction); 
             
            stab_driver0.VACTUAL(vactual_speed);
            sg_raw = stab_driver0.SG_RESULT();
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0);
            //Serial.print(" S0 Down ");
            break;  
                 
        case 5: // move stabilizer 1 up
        
            stab_driver1.shaft(!shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            //Serial.print(" S1 Up ");
            break;
            
        case 6: // move stabilizer 1 down
        
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            //Serial.print(" S1 Down ");
            break;
            
        case 7: // move stabilizer 2 up
        
            stab_driver2.shaft(!shaft_direction); 
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed);
            sg_raw = stab_driver2.SG_RESULT();
            //Serial.print(" S2 Up ");
            break;
        case 8: // move stabilizer 2 down
            stab_driver2.shaft(shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed);
            sg_raw = stab_driver2.SG_RESULT();
            //Serial.print(" S2 Down ");
            break;       
        case 9: // move stabilizer 0 and stabilizer 1 up
        
            stab_driver0.shaft(!shaft_direction); 
            stab_driver1.shaft(!shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0);
            //Serial.print(" S0 & S1 up");
            break;        
        case 10: // move stabilizer 0 and stabilizer 1 down
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0); 
            //Serial.print(" S0 & S1 down");
            break;
    }
    return sg_raw;
} 

void wheel_move(byte dir)
{   
//    if (dir == 0) {current_delay = 0;}
//    else          {dir = wheel_accelerate(dir);}
    
    current_delay = 1200;
    
    switch(dir)
    {   // Switch between priority directions
        case 0: // all stop
          step_wheel(0,0);
          delayMicroseconds(current_delay);
          //Serial.print(" Stop ");
          break; 
        case 1: // Forward sequence 1100 
          step_wheel(B10011111, B10010000);
          delayMicroseconds(current_delay);
          //Serial.print(" Forward ");
          break; 
        case 2: // Backward sequence 0011
          step_wheel(B01101111, B01100000);
          delayMicroseconds(current_delay);
          //Serial.print(" Backward ");
          break;
        case 3: //Left sequence 1001
          step_wheel(B11001111, B11000000);
          delayMicroseconds(current_delay);
          //Serial.print(" Left ");
          break;
        case 4: //Right sequence 0110
          step_wheel(B00111111, B00110000);
          delayMicroseconds(current_delay); 
          //Serial.print(" Right ");
          break;
        case 5: //-rz sequence 0000
          step_wheel(B11111111, B11110000);
          delayMicroseconds(current_delay);
          //Serial.print(" Clockwise ");
          break;    
        case 6: //+rz sequence 1111
          step_wheel(B00001111, B00000000);
          delayMicroseconds(current_delay);
          //Serial.print(" Anticlockwise ");
          break;     
    }
//    prev_dir = dir;
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
    delayMicroseconds(12);

    // LOW pulse 
    digitalWrite(latch_pin, LOW);
    shiftOut(data_pin, clock_pin, MSBFIRST, step_low);
    digitalWrite(latch_pin, HIGH);
}
