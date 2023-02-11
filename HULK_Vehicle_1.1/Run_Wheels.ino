// determine movement function takes analog joystick inputs and determines what to do. 
// This function is capable of moving wheels using both communication protocols (UART & STEP-DIR)
// all wheels move simulataneously
void move_all_wheel(){
  // Variables for net wheel direction
  const byte stop_moving          = 0;
  const byte move_forward         = 1;
  const byte move_backward        = 2;
  const byte move_left            = 3;
  const byte move_right           = 4;
  const byte rotate_clockwise     = 5;
  const byte rotate_anticlockwise = 6;
  const byte deadzone             = 30;
  const int max_joy_value         = 512;
  
  // One joystick 0 controls rotation about the center
  // Check is joystick is out of the deadzone
  if (abs(joy1_y_input) > deadzone)
  { 
    joy_norm = abs(joy1_y_input/max_joy_value);
    vactual_speed = joy_norm * max_vactual;           
    microseconds_per_step = min_microseconds_per_step / joy_norm;
    
    if (joy1_y_input < 0){wheel_move(rotate_clockwise);    }  
    else                 {wheel_move(rotate_anticlockwise);}
  }
  // joystick 1 controls translation
  // check to see if x or y input is out of the deadzone
  else if (abs(joy0_x_input) > deadzone | abs(joy0_y_input) > deadzone)
  {
    // Motion is prioritized to x OR y based on greatest value
    // priority x direction
    if (abs(joy0_x_input) > abs(joy0_y_input))
    {     
      joy_norm = abs(joy0_x_input/max_joy_value);
      vactual_speed = joy_norm * max_vactual;           
      microseconds_per_step = min_microseconds_per_step / joy_norm;
      if (joy0_x_input > 0){wheel_move(move_backward);}
      else                 {wheel_move(move_forward); }  
    }
    // Priority y direction
    if (abs(joy0_x_input) < abs(joy0_y_input))
    {  
      joy_norm = abs(joy0_y_input/max_joy_value);
      vactual_speed = joy_norm * max_vactual;           
      microseconds_per_step = min_microseconds_per_step / joy_norm;
      if (joy0_y_input > 0){wheel_move(move_left); } 
      else                 {wheel_move(move_right);}     
    }
  }
  // When neither joysticks are out of the deadzone, the wheels stop
  else{wheel_move(stop_moving);} // no movement
}

// Move individual wheel, one joystick controls movement 
// Use switch_between to control different wheels 
void move_indiv_wheel(){
  // Variables for direction
  const byte stop_moving          = 0;
  const byte move_forward         = 1;
  const byte move_backward        = 2;
  const byte move_left            = 3;
  const byte move_right           = 4;
  const byte rotate_clockwise     = 5;
  const byte rotate_anticlockwise = 6;
  const byte deadzone             = 30;
  const int max_joy_value         = 512;
  // Set booleans

  // Check to see if joystick has moved enough to register input 
  if (abs(joy1_x_input) > deadzone)
  { 
    joy_norm = abs(joy1_x_input/max_joy_value);
    vactual_speed = joy_norm * max_vactual;           
    microseconds_per_step = min_microseconds_per_step / joy_norm;
    
    if (joy1_x_input > 0){wheel_move(6 + 2*now_controlling + move_forward);}  
    if (joy1_x_input < 0){wheel_move(6 + 2*now_controlling + move_backward);}         
  }
  // Both joysticks in deadzone means the stabilizers will stop
  else {wheel_move(stop_moving);} 
}

// Enable the wheel drivers for manual control 
// Will also disable the stabilizer drivers 
void enable_wheels(){
  // Enable wheel drivers, LOW = active.  
  // Addresses can ALL be high or low, they are hardwired together 
  // Stabilizer nable pin is controlled by 7th bit sent to stabilizer shift register, set high with stabilizer_en_pin = 64 
  // Must turn off stabilizer drivers first, ensure stabilizer control by setting flags  
  control_stabilizers = true;
  control_wheels = false;
  byte stabilizer_en_pin = 128;
  shift_data = stabilizer_en_pin; // Sets enable pins high on stabilizers    
  send_stepdir_data();// Send shift data to stability system
  // Give control back to wheels with flags 
  control_stabilizers = false;
  control_wheels = true;
  
  const byte wheel_en_pin = 4;
  
  digitalWrite(wheel_en_pin, LOW); // Set wheel enable pin to high, disabling wheel drivers 

  manual_control = true;
}


// Function handles all wheel movement, will move wheels simultaneously or indivdually depending on case given 
// This function is used when in manual control 
// Function can handle both UART and STEP-DIR interfacing 
// STEP-DIR IS NOT COMPLETE: INDIVIDUAL WHEEL CONTROL IS NOT IMPLEMENTED 
void wheel_move(int dir){       
  // Check for interface type 
  if(use_uart)
  {
    bool shaft_direction; // true turns motors forward, false turns motors backward. 
    switch(dir)// Switch between priority directions
    {
      case 0: // all stop 
        vactual_speed = 0;
        send_uart_data();
        Serial.print(" Stop ");
        break; 
      case 1: // Forward sequence 1100 
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);  
        send_uart_data();
        Serial.print(" Forward ");
        break; 
      case 2: // Backward sequence 0011
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  
        wheel_driver1.shaft(!shaft_direction);  
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(shaft_direction); 
        send_uart_data();
        Serial.print(" Backward ");
        break;
      case 3: //Left sequence 1001
        // Set motor directions
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(shaft_direction);  
        send_uart_data();
        Serial.print(" Left ");
        break;
      case 4: //Right sequence 0110
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);   
        send_uart_data(); 
        Serial.print(" Right ");
        break;
      case 5: //-rz sequence 0000
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        wheel_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);  
        send_uart_data(); 
        Serial.print(" Clockwise ");
        break;    
      case 6: //+rz sequence 1111
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(shaft_direction);  
        send_uart_data();
        Serial.print(" Anticlockwise ");
        break;      
      case 7: //0 Forward
        wheel_driver0.shaft(shaft_direction); 
        wheel_driver0.VACTUAL(vactual_speed);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W0 Forward ");
        break;    
      case 8: //0 Backward
        wheel_driver0.shaft(!shaft_direction); 
        wheel_driver0.VACTUAL(vactual_speed);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W0 Backward ");
        break;
      case 9: //1 Forward
        wheel_driver1.shaft(shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(vactual_speed);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W1 Forward ");
        break;      
      case 10: //1 Backward
        wheel_driver1.shaft(!shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(vactual_speed);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W1 Backward ");
        break;
      case 11: //2 Forward
        wheel_driver2.shaft(shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(vactual_speed);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W2 Forward ");
        break;
      case 12: //2 Backward
        wheel_driver2.shaft(!shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(vactual_speed);
        wheel_driver3.VACTUAL(0);
        Serial.print(" W2 Backward ");
        break;
      case 13: //3 Forward
        wheel_driver3.shaft(shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(vactual_speed);
        Serial.print(" W3 Forward ");
        break;
      case 14: //3 Backward
        wheel_driver3.shaft(!shaft_direction); 
        wheel_driver0.VACTUAL(0);
        wheel_driver1.VACTUAL(0);
        wheel_driver2.VACTUAL(0);
        wheel_driver3.VACTUAL(vactual_speed);
        Serial.print(" W3 Backward ");
        break;
    }
  }
  if (use_stepdir)
  {
    // Bit references for the shift register, identifying register pin outputs 
    const byte dir0    = 1;    //00000001
    const byte step0   = 2;    //00000010
    const byte dir1    = 4;    //00000100
    const byte step1   = 8;    //00001000
    const byte dir2    = 16;   //00010000
    const byte step2   = 32;   //00100000
    const byte dir3    = 64;   //01000000
    const byte step3   = 128;  //10000000
    
    switch(dir)// Switch between priority directions
    {
      case 0: //stop 
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Stop ");
        break; 
      case 1:   //+ty sequence 1100 
        // Set step high
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir1;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir1;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Forward ");
        break;
      case 2: //-ty sequence 0011
        shift_data = step0 + step1 + step2 + step3 + dir2 + dir3;
        send_stepdir_data();
        shift_data = dir2 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Backward ");
        break;
      case 3: //+txsequence 1001
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir3;
        send_stepdir_data();
        shift_data = dir0 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Left ");
        break;
      case 4: //-tx sequence 0110
        shift_data = step0 + step1 + step2 + step3 + dir1 + dir2;
        send_stepdir_data();
        shift_data = dir1 + dir2;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Right ");
        break;
      case 5: //-rz sequence 0000
        shift_data = step0 + step1 + step2 + step3;
        send_stepdir_data();
        // set step low
        shift_data = 0;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Clockwise ");
        break;
      case 6: //+rz sequence 1111
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir1 + dir2 + dir3;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir1 + dir2 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        Serial.print(" Anticlockwise ");
        break; 
    }
  }
}  
