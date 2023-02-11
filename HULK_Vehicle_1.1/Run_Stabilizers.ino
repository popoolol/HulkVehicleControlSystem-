// Enable the stabilizer drivers for manaul control 
// Will also disable the wheel drivers 
void enable_stabilizers(){
  // Enable stabilizers drivers, LOW = active.  
  // Addresses can ALL be high or low, they are hardwired together 
  // Enable pin is controlled by 7th bit sent to shift register, set high with stabilizer_en_pin = 64 
  // First set proper flags 
  control_stabilizers = true;
  control_wheels = false;

  byte address_pin = 128;
  shift_data = address_pin;     // Sending 0 to shift register sets ALL registers low, including the stabilizer enable pin   
  send_stepdir_data();// Send shift data to stability system
  
  byte wheel_en_pin = 4;
  digitalWrite(wheel_en_pin, HIGH); // Set wheel enable pin to high, disabling wheel drivers 

  manual_control = true;
}

// determine movement function takes analog joystick inputs and determines what to do. 
// This function is capable of moving stabilizers using both communication protocols (UART & STEP-DIR)
// all stabilizers move simulataneously

void move_all_stabilizer(){
  // Variables for stabilizer movement 
  const byte deadzone           = 30; // Deadzone is a region where the joystick inputs are ignored 
  const byte hold_stabilizers   = 0;
  const byte raise_stabilizers  = 1;
  const byte lower_stabilizers  = 2;

  vactual_speed         = 80000; // Run at constant speed
  microseconds_per_step = 200;
  
  // Set booleans
  all_stabilizers_home = false; // manual mode resets the flag saying the stabilizers have not been homed
  
  // stabilizer logic, both joysticks perform only up ad down movement
  // joystick inputs contoll ALL stabilizers simultaneously! Cannot be independent
  // Check to see if either joystick is out of deadzone
  if (abs(joy0_x_input) > deadzone | abs(joy1_x_input) > deadzone){ 
    // Check to see which direction is desired 
    if (abs(joy0_x_input) > abs(joy1_x_input)){    
      if (joy0_x_input > 0){stabilizer_move(lower_stabilizers);} 
      if (joy0_x_input < 0){stabilizer_move(raise_stabilizers);}  
    }
    if (abs(joy0_x_input) < abs(joy1_x_input)){    
      if (joy1_x_input > 0){stabilizer_move(lower_stabilizers);}  
      if (joy1_x_input < 0){stabilizer_move(raise_stabilizers);}        
    }
  }
  // Both joysticks in deadzone means the stabilizers will stop
  else{
    stabilizer_move(hold_stabilizers);
  } 
}

// Move individual stabilizer, one joystick controls movement 
// Use switch_between to control different stabilizers 
void move_indiv_stabilizer(){
  // Variables for stabilizer movement 
  const byte deadzone           = 30; // Deadzone is a region where the joystick inputs are ignored 
  const byte hold_stabilizers   = 0;
  const byte raise_stabilizers  = 1;
  const byte lower_stabilizers  = 2;

  vactual_speed         = 80000; // Run at constant speed
  microseconds_per_step = 200;

  // Set booleans
  all_stabilizers_home = false; // resets the flag saying the stabilizers have not been homed

  // Check to see if joystick has moved enough to register input 
  if (abs(joy1_x_input) > deadzone){ 
    if (joy1_x_input > 0){stabilizer_move(2 + 2*now_controlling + lower_stabilizers);}  
    if (joy1_x_input < 0){stabilizer_move(2 + 2*now_controlling + raise_stabilizers);}         
  }
  // Both joysticks in deadzone means the stabilizers will stop
  else{
    stabilizer_move(hold_stabilizers);
  } 
}


// Move Stabilizer handles all stabilizer movement, manual or automatic
// * UART and Step-direction 
// * Controlling all at once, controlling individual 
// Broken into two main parts, uart and stepdir
// Both parts have the same number of cases, and the cases are the same for the two protocols 
// The vactual speed, or microstep_per_second is set in the function that calls move_stabilizer
void stabilizer_move(int dir){       

  // Check interface type first case is uart (vactual) control 
  if(use_uart)
  { 
    bool shaft_direction; // true turns motors forward, false turns motors backward. 
    switch(dir)// Switch between priority directions
    {
      case 0: // all stop 
        vactual_speed = 0;
        send_uart_data();
        sg_return = 0;
        Serial.print(" ALL STOP ");
        break; 
      case 1: //All up
        // Set motor directions
        stab_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        stab_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        stab_driver2.shaft(!shaft_direction);    
        send_uart_data();
        Serial.print(" All Up ");
        break;
      case 2: // All down
        stab_driver0.shaft(shaft_direction);   // shaft is set by boolean
        stab_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        stab_driver2.shaft(shaft_direction);     
        send_uart_data();
        Serial.print(" All Down ");
        break;      
      case 3: // move stabilizer 0 up
        stab_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(vactual_speed);
        sg_return = stab_driver0.SG_RESULT();
        stab_driver1.VACTUAL(0);
        stab_driver2.VACTUAL(0); 
        Serial.print(" S0 Up ");
        break; 
      case 4: // move stabilizer 0 down
        stab_driver0.shaft(shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(vactual_speed);
        sg_return = stab_driver0.SG_RESULT();
        stab_driver1.VACTUAL(0);
        stab_driver2.VACTUAL(0);
        Serial.print(" S0 Down ");
        break;       
      case 5: // move stabilizer 1 up
        stab_driver1.shaft(!shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(0);
        stab_driver1.VACTUAL(vactual_speed);
        sg_return = stab_driver1.SG_RESULT();
        stab_driver2.VACTUAL(0);
        Serial.print(" S1 Up ");
        break;
      case 6: // move stabilizer 1 down
        stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(0);
        stab_driver1.VACTUAL(vactual_speed);
        sg_return = stab_driver1.SG_RESULT();
        stab_driver2.VACTUAL(0);
        Serial.print(" S1 Down ");
        break;
      case 7: // move stabilizer 2 up
        stab_driver2.shaft(!shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(0);
        stab_driver1.VACTUAL(0);
        stab_driver2.VACTUAL(vactual_speed);
        sg_return = stab_driver2.SG_RESULT();
        Serial.print(" S2 Up ");
        break;
      case 8: // move stabilizer 2 down
        stab_driver2.shaft(shaft_direction);  // shaft is set by boolean
        stab_driver0.VACTUAL(0);
        stab_driver1.VACTUAL(0);
        stab_driver2.VACTUAL(vactual_speed);
        sg_return = stab_driver2.SG_RESULT();
        Serial.print(" S2 Down ");
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
  }

  // This case is for step-direction interface
  if (use_stepdir)
  {
    // Bit references for the shift register, identifying register pin outputs 
    // Registers 0-6 are identical between systems. 7-8 are different. The movement systems needs 7-8 to control the fourth motor. The stability 
    // system only has 3 motors, leaving those pins open. They are the enable pin and the address pin for the stability system.
    // The enable pin is a dedicated arduino pin on the movement system, and the MS1 and MS2 addresses are set high on the movement system using a jumper pin on the cicuit. 
    // Setting the 7-8 bit on the stabilty system will set the MS1 and MS2 pins high, designating driver addresses, as well as setting the enable pin
    const byte dir0               = 1;  //00000001
    const byte step0              = 2;  //00000010
    const byte dir1               = 4;  //00000100
    const byte step1              = 8;  //00001000
    const byte dir2               = 16; //00010000
    const byte step2              = 32; //00100000
    const byte stabilizer_en_pin  = 64; //01000000
    const int address_pin         = 128;//10000000
    
    switch(dir)// Switch between priority directions
    {
      case 0: // all stop (hold)
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break;        
      case 1: //-rz sequence 0000
        shift_data = step0 + step1 + step2;
        send_stepdir_data();
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break;        
      case 2: //+rz sequence 1111
        shift_data = step0 + step1 + step2 + dir0 + dir1 + dir2 ;
        send_stepdir_data();
        shift_data = dir0 + dir1 + dir2;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break;          
      case 3: // move stabilizer 0 up
        shift_data = step0 + address_pin;
        send_stepdir_data();
        shift_data = address_pin;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step); 
        break;
      case 4: // move stabilizer 0 down
        shift_data = step0 + dir0 + address_pin;
        send_stepdir_data();
        shift_data = dir0 + address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;       
      case 5: // move stabilizer 1 up
        shift_data = step1 + address_pin;
        send_stepdir_data();
        shift_data = address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;
      case 6: // move stabilizer 1 down
        shift_data = step1 + dir1 + address_pin;
        send_stepdir_data();
        shift_data = dir1 + address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;
      case 7: // move stabilizer 2 up
        shift_data = step2 + address_pin;
        send_stepdir_data();
        shift_data = address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;
      case 8: // move stabilizer 2 down
        shift_data = step2 + dir2 + address_pin;
        send_stepdir_data();
        shift_data = dir2 + address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;        
      case 9: // move stabilizer 0 and stabilizer 1 up
        shift_data = step0 + step1 + address_pin;
        send_stepdir_data();
        shift_data = address_pin;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step); 
        break;       
      case 10: // move stabilizer 0 and stabilizer 1 down
        shift_data = step0 + step1 + dir0 + dir1 + address_pin;
        send_stepdir_data();
        shift_data = dir0 + dir1 + address_pin;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
        break;        
    }
  }
}  
