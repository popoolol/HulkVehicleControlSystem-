// Gets joystick inputs: analog values in x and y and the buttons
// The buttons are toggled. joystick buttons change between controlling movement and controlling stability 
// pressing both buttons at once will set into autoleveling mode 
void read_joysticks(){
  // Begin by polling the joysticks for input
  joy0_x_input = map(analogRead(joy0_x_pin), 0, 1024, -max_joy_value, max_joy_value) - joy0_x_shift; // Read x direction, remap to have negative values
  joy0_y_input = map(analogRead(joy0_y_pin), 0, 1024, -max_joy_value, max_joy_value) - joy0_y_shift; // Read y direction, remap to have nagative values 
  joy1_x_input = map(analogRead(joy1_x_pin), 0, 1024, -max_joy_value, max_joy_value) - joy1_x_shift;                
  joy1_y_input = map(analogRead(joy1_y_pin), 0, 1024, -max_joy_value, max_joy_value) - joy1_y_shift; 
  button0_state = digitalRead(joy0_b_pin);
  //button1_state = digitalRead(joy1_b_pin);
  button1_state = true;
  // Logic to switch between connection interace and system
  // Switching is controlled by toggling
  if (manual_control){
//    if (!button0_state & button1_state) { // press button 0 and not button 1
//      if (millis() - last_single_button_time > debounce_time){
//        if    (control_stabilizers){enable_wheels();} // flip flop between controlling wheels and stability
//        else                       {enable_stabilizers();}
//        last_single_button_time = millis(); // reset button time
//      }
//    } 
    if (!button0_state & button1_state) { // press button 1 and not button 0
//      if (millis() - last_single_button_time > debounce_time){ 
//        if    (use_uart){enable_stepdir();} // flip flop between control interfaces
//        else            {enable_uart();}
//        last_single_button_time = millis();
//      }
      use_uart = false;
      use_stepdir = false;
      control_stabilizers = false;
      control_wheels = false;
      manual_control = false;
      run_autoleveling = true;
      home_stabilizers();
    }
    // Pass to manual movement function to process movement decisions

//    Serial.print(" x0: ");Serial.print(joy0_x_input);
//    Serial.print(" y0: ");Serial.print(joy0_y_input);
//    Serial.print(" x1: ");Serial.print(joy1_x_input);
//    Serial.print(" y1: ");Serial.print(joy1_y_input);
    Serial.print(" b0: ");Serial.print(button0_state);
    Serial.print(" b1: ");Serial.print(button1_state);
    Serial.print(" MC: ");Serial.print(manual_control);
    Serial.print(" S: ");Serial.print(control_stabilizers);
    Serial.print(" W: ");Serial.print(control_wheels);
    Serial.print(" UA: ");Serial.print(use_uart);
    Serial.print(" SD: ");Serial.print(use_stepdir);
//    Serial.print(" A: ");Serial.print(run_autoleveling);

    manual_movement();
  }
  Serial.print(" SG0: ");
  Serial.print(stab_driver0.SG_RESULT());
//  Serial.print(stab_driver0.cs_actual());
//  Serial.print(stab_driver0.SGTHRS());
  Serial.print(" SG1: ");
  Serial.print(stab_driver1.cs_actual());
//  Serial.print(stab_driver1.SG_RESULT());
//  Serial.print(stab_driver1.SGTHRS());
  Serial.print(" SG2: ");
//  Serial.print(stab_driver2.cs_actual());
//  Serial.print(stab_driver2.SG_RESULT());
  Serial.print(stab_driver2.SGTHRS());

  Serial.print(" I RUN0: ");
  Serial.print(stab_driver0.irun());
//  Wire.beginTransmission(accel_register);
//  Wire.write(0x3B);  
//  Wire.endTransmission(false);
//  Wire.requestFrom(accel_register,6,true);  //Reading 6 bytes from MPU
//  accel_x_input = Wire.read()<<8|Wire.read() - accel_x_shift;    
//  accel_y_input = Wire.read()<<8|Wire.read() - accel_y_shift;
//
//  Serial.print(" X = "); Serial.print(accel_x_input);
//  Serial.print(" Y = "); Serial.print(accel_y_input);
//  Serial.print(" X shift = "); Serial.print(accel_x_shift);
//  Serial.print(" Y shift = "); Serial.print(accel_y_shift);
  Serial.println();
  
  if (!button0_state & !button1_state) { // press both buttons simultaneously
    // Check to see if auto leveled  
    if (!run_autoleveling){
      //enable_autoleveling();
      
      //Serial.print(" ABOUT TO AUTOLEVEL! ");
      //autoleveling(); // run autoleveling function. Will run in loop until leveling completes
    }
    
//    if (run_autoleveling){
//      return_home();
//      use_uart = true;          // no longer using uart 
//      use_stepdir = false;      // no longer using this step-direction protocol  
//      run_autoleveling = false; // set auto leveling operation flag 
//      manual_control = true;   // remove manual control from the system
//      control_wheels = true;
//      control_stabilizers = false;
//      all_stabilizers_home = true;
//      
//    }
  }
}

// determine movement function takes analog joystick inputs and determines what to do. 
// This function is capable of moving wheels and stabilizers using both communication protocols (UART & STEP-DIR)
// When in manual control, all wheels move simulataneously, all stabilizers move simultaneously 

void manual_movement(){
  // Variables for net wheel direction
  int move_forward = 1;
  int move_backward = 2;
  int move_left = 3;
  int move_right = 4;
  int rotate_clockwise = 5;
  int rotate_anticlockwise = 6;
  int stop_moving = 7;
  // Variables for stabilizer movement 
  int raise_stabilizers = 1;
  int lower_stabilizers = 2;
  int hold_stabilizers = 3;

  // check if wheels or stabilizers are desired to control   
  if (control_wheels){
    // One joystick 0 controls rotation about the center
    // Check is joystick is out of the deadzone
    if (abs(joy1_y_input) > deadzone){ 
      joy_norm = abs(joy1_y_input/max_joy_value);
      if (joy1_y_input < 0){wheel_move(rotate_clockwise);}  
      else                 {wheel_move(rotate_anticlockwise);}
    }
    // joystick 1 controls translation
    // check to see if x or y input is out of the deadzone
    else if (abs(joy0_x_input) > deadzone | abs(joy0_y_input) > deadzone){
      // Motion is prioritized to x OR y based on greatest value
      // priority x direction
      if (abs(joy0_x_input) > abs(joy0_y_input)){     
        joy_norm = abs(joy0_x_input/max_joy_value);
        if (joy0_x_input > 0){wheel_move(move_backward);}
        else                 {wheel_move(move_forward);}  
      }
      // Priority y direction
      if (abs(joy0_x_input) < abs(joy0_y_input)){  
        joy_norm = abs(joy0_y_input/max_joy_value);
        if (joy0_y_input > 0){wheel_move(move_left);} 
        else                 {wheel_move(move_right);}     
      }
    }
    // When neither joysticks are out of the deadzone, the wheels stop
    else{joy_norm = 0;wheel_move(stop_moving);} // no movement
  }
  
  // Check for stabilizer control 
  if (control_stabilizers){

    all_stabilizers_home = false; // manual mode resets the flag saying the stabilizers have not been homed
    
    // stabilizer logic, both joysticks perform only up ad down movement
    // joystick inputs contoll ALL stabilizers simultaneously! Cannot be independent
    // Check to see if either joystick is out of deadzone
    if (abs(joy0_x_input) > deadzone | abs(joy1_x_input) > deadzone){ 
      // Check to see which direction is desired 
      if (abs(joy0_x_input) > abs(joy1_x_input)){    
        joy_norm = abs(joy0_x_input/max_joy_value);
        if (joy0_x_input > 0){stabilizer_move(lower_stabilizers);} 
        if (joy0_x_input < 0){stabilizer_move(raise_stabilizers);}  
      }
      if (abs(joy0_x_input) < abs(joy1_x_input)){    
        joy_norm = abs(joy1_x_input/max_joy_value);
        if (joy1_x_input > 0){stabilizer_move(lower_stabilizers);}  
        if (joy1_x_input < 0){stabilizer_move(raise_stabilizers);}        
      }
    }
    // Both joysticks in deadzone means the stabilizers will stop
    else{joy_norm = 0; stabilizer_move(hold_stabilizers);} // no movement
  }
}
