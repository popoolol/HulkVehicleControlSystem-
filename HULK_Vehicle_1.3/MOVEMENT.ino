// Move Sta0bilizer handles all sta0bilizer movement, manual or automatic
// * UART and Step-direction 
// * Controlling all at once, controlling individual 
// 0broken into two main parts, uart and stepdir
// 0both parts have the same num0ber of cases, and the cases are the same for the two protocols 
// The vactual speed, or microstep_per_second is set in the function that calls move_sta0bilizer
byte stabilizer_move(byte dir){       

    // Default stallGaurd value, when not asked to pull it 
    byte sg_raw = 255; 

    uint32_t vactual_speed = 80000;
    
    // The sta0bilizer movement speed is constant. There is no real need to have a varia0ble speed
        
    switch(dir)// Switch 0between priority directions
    {   
        // The cases set the motor direction with shaft(), then sets the speed with VACTUAL()
        // Cases 0 - 2 mave all sta0bilizers simulataneously 
        // Cases 3 - 8 move sta0bilizers individually 
        // Cases 9 -10 move sta0bilizers 0 and 1 together (for leveling)
        case 0: // all stop 
        
            vactual_speed = 0;
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            break; 
            
        case 1: // All Up
            shaft_direction = true;
            
            stab_driver0.shaft(shaft_direction); // shaft sets motor direction
            stab_driver1.shaft(shaft_direction); // !shaft_direction reverses direction
            stab_driver2.shaft(shaft_direction); 
               
            stab_driver0.VACTUAL(vactual_speed);  // Set speed 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            break;
            
        case 2: // All down
            shaft_direction = false;
            
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);  
            stab_driver2.shaft(shaft_direction);     
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            break;    
              
        case 3: // move stabilizer 0 up
            shaft_direction = true;
            // Cases 3 - 8 also retrive the result of the stall gaurd for sensorless homing 
            stab_driver0.shaft(shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);  // SG_RESULT() gets the stallgaurd value
            sg_raw = stab_driver0.SG_RESULT(); // This value is called IMMEDIATELY after movement for 0best results 
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0); 
            break; 
            
        case 4: // move stabilizer 0 down
            shaft_direction = false;
            stab_driver0.shaft(shaft_direction); 
             
            stab_driver0.VACTUAL(vactual_speed);
            sg_raw = stab_driver0.SG_RESULT();
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0);
            break;  
                 
        case 5: // move stabilizer 1 up
            shaft_direction = true;
            stab_driver1.shaft(shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            break;
            
        case 6: // move stabilizer 1 down
            shaft_direction = false;
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            break;
            
        case 7: // move stabilizer 2 up
            shaft_direction = true;
            stab_driver2.shaft(shaft_direction); 
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed);
            sg_raw = stab_driver2.SG_RESULT();
            break;
            
        case 8: // move stabilizer 2 down
            shaft_direction = false;
            stab_driver2.shaft(shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed);
            sg_raw = stab_driver2.SG_RESULT();
            break;  
                 
        case 9: // move stabilizer 0 and stabilizer 1 up
            shaft_direction = true;
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0);
            break;        
            
        case 10: // move stabilizer 0 and stabilizer 1 down
            shaft_direction = false;
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0); 
            break;
    }
    
    if (bitRead(auto_bool, 3))
    { 
        byte sg_next_index = loop_counter % 3;
      
        if (sg_raw == 0) {sg_raw = 255;}
        
        sg_running_average[sg_next_index] = sg_raw;
        
        byte sg_smooth = 0;
      
        for(int idx = 0; idx < 3; ++idx)
        {
            sg_smooth += sg_running_average[idx] / 3;
        }    
        return sg_smooth;
    }
    else {return;}
} 

void wheel_move(byte dir)
{      
    uint16_t target_delay;
    uint16_t min_delay;
    uint16_t acc_val;   //Allows for a variable acceleration value 
    uint16_t decc_val = 250;  //Sets the decceleration value. Currently a constant
    uint16_t max_delay = 16000; //The maximum delay which is the starting speed when moving in a new direction
    
    //min_delay is the max speed. This sets the max speed for forwards and backwards to be twice as fast as rotation or transverse movement
    if (dir == 1 | dir == 2) {min_delay = 400;} 
    else                     {min_delay = 800;}

    //This checks if the last delay was 0 (meaning it had been not moving) or if there was a change in direction (and the new direction
    //is not zero) then set the delay to the maximum value (lowest speed)
    if (((current_delay == 0) | (dir != last_dir)) & dir != 0) {current_delay = max_delay;}

    //If the current direction is 0 (not moving) Then sets the target delay as the maximum delay. Used for deccelerating. Or else sets 
    //the target as the min_delay
    if(dir == 0) {target_delay = max_delay;} 
    else         {target_delay = min_delay;}
    
    //Used to scale the acceleration. Starts out with slow acceleration to allow for fine tuning, then goes to medium acceleration, then fast
    if      (current_delay >= 15000) {acc_val = 10; } 
    else if (current_delay >= 14000) {acc_val = 50; } 
    else                             {acc_val = 100;}

    //Sets the current_delay based on the acceleration/decceleration values. If the magnitude of the difference between the target delay and
    //the current delay is larger than the acceleration/decceleration value, then sets the current delay equal to the current_delay
    // +/- the acceleration/decceleration value. If the current minus the target is less than the value, then it sets the current_delay to 
    //equal the target delay
    if(current_delay > target_delay)
    {
        if((current_delay - target_delay) > acc_val) {current_delay = current_delay - acc_val;}
        else                                         {current_delay = target_delay;           }
    } 
    else if(current_delay < target_delay)
    {
        if((target_delay - current_delay) > decc_val) {current_delay = current_delay + decc_val;} 
        else                                          {current_delay = target_delay;            }
    }
    
    //If the current direction is not zero, then the last_dir variable is set to the current direction. If current_delay equals max_delay (therefore
    // the vehicle is deccelerating and hit the slowest speed) then the last_dir is set to 0. This is used for decceleration
    if(dir != 0 ) {last_dir = dir;} 
    else          
    {
        if(current_delay == max_delay) {last_dir = 0;}    
    }
    
    switch(dir)
    {   // Switch 0between priority directions
        case 0: // all stop
          current_delay = 0;
          step_wheel(0,0);
          delayMicroseconds(current_delay);
          break; 
        case 1: // Forward sequence 1100 Forward
          step_wheel(0b10011111, 0b10010000);
          delayMicroseconds(current_delay);
          break; 
        case 2: // 0backward sequence 0011 Backward
          step_wheel(0b01101111, 0b01100000);
          delayMicroseconds(current_delay);
          break;
        case 3: //Left sequence 1001 Left
          step_wheel(0b11001111, 0b11000000);
          delayMicroseconds(current_delay);
          break;
        case 4: //Right sequence 0110 Right
          step_wheel(0b00111111, 0b00110000);
          delayMicroseconds(current_delay); 
          break;
        case 5: //-rz sequence 0000 Clockwise
          step_wheel(0b11111111, 0b11110000);
          delayMicroseconds(current_delay);
          break;    
        case 6: //+rz sequence 1111 Anticlockwise
          step_wheel(0b00001111, 0b00000000);
          delayMicroseconds(current_delay);
          break;     
    }
}


void step_wheel(uint8_t step_high, uint8_t step_low)
{
    /* 
    0bit Reference for step and direction pins to wheel drivers 
    const byte step0 = 0b00000001;
    const byte step1 = 0b00000010;
    const byte step2 = 0b00000100;
    const byte step3 = 0b00001000;
    const byte dir0  = 0b00010000;
    const byte dir1  = 0b00100000;
    const byte dir2  = 0b01000000;
    const byte dir3  = 0b10000000;
    */
    
    // Define shift register pins
    const byte data_pin  = 8;
    const byte latch_pin = 9;
    const byte clock_pin = 10;

    // Set pin modes 
    pinMode(latch_pin,OUTPUT);
    pinMode(data_pin, OUTPUT);
    pinMode(clock_pin,OUTPUT);

    // Need to push the data through LCD register into Wheel register 
    uint8_t push_byte = 0b00000000;

    /* 
      Set latch LOW to load data into register
      Send data to shift register with shiftOut 
      Set latch HIGH to shift data out of register 
     */
    // HIGH pulse 
    digitalWrite(latch_pin, LOW);
    shiftOut(data_pin, clock_pin, MSBFIRST, step_high);
    shiftOut(data_pin, clock_pin, MSBFIRST, push_byte); // This pushes the information through the first shift register
    digitalWrite(latch_pin, HIGH);
    
    // Delay 
    delayMicroseconds(12);

    // LOW pulse 
    digitalWrite(latch_pin, LOW);
    shiftOut(data_pin, clock_pin, MSBFIRST, step_low);
    shiftOut(data_pin, clock_pin, MSBFIRST, push_byte);
    digitalWrite(latch_pin, HIGH);
}
