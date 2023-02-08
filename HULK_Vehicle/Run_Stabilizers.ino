// Enable autoleveling sets proper flags for auto leveling procedure 
void enable_autoleveling(){
  // Remove all manual control
  manual_control = false;   // remove manual control from the system
  control_wheels = false;
  control_stabilizers = false;
  // Enable step-direction protocol 
  enable_stepdir();
  // Set flag for autoleveling
  run_autoleveling = true; // auto leveling operation flag 

}

// Enable the stabilizer drivers for manaul control 
// Will also disable the wheel drivers 
void enable_stabilizers(){
  // Enable stabilizers drivers, LOW = active.  
  // Addresses can ALL be high or low, they are hardwired together 
  // Enable pin is controlled by 7th bit sent to shift register, set high with stabilizer_en_pin = 64 
  // First set proper flags 
  control_stabilizers = true;
  control_wheels = false;
  
  shift_data = 0;     // Sending 0 to shift register sets ALL registers low, including the stabilizer enable pin   
  send_stepdir_data();// Send shift data to stability system
  
  digitalWrite(wheel_en_pin, HIGH); // Set wheel enable pin to high, disabling wheel drivers 

  manual_control = true;
}

// Stab 3 is for moving stabilizers in manual mode, there are 3 movements including no movement 
// Function can handle both uart and step-direction inputs. 
// The maximum speeds of these systems is set in STEP-DIRECTION and UART tabs 
void stabilizer_move(int dir){       

  // Check interface type first case is uart (vactual) control 
  if(use_uart){

    vactual_speed = joy_norm * max_vactual; // joystick input scaled to RPM (kinda sloppy)
    
    switch(dir){// Switch between priority directions
      case 1: //-rz sequence 0000
        // Set motor directions
        stab_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        stab_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        stab_driver2.shaft(!shaft_direction);   
        // Set vactual_speed of motor drivers 
        send_uart_data();
        break;
      case 2: //+rz sequence 1111
        stab_driver0.shaft(shaft_direction);   // shaft is set by boolean
        stab_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        stab_driver2.shaft(shaft_direction);     
        // Set vactual_speed of motor drivers 
        send_uart_data();
        break;
      case 3: //stop 
        send_uart_data();
        break;  
    }
  }

  // This case is for step-direction interface
  if (use_stepdir){

    microseconds_per_step = 37500/max_stabilizer_rpm;

    switch(dir){// Switch between priority directions
      case 1: //-rz sequence 0000
        shift_data = step0 + step1 + step2;
        send_stepdir_data();
        // set step low
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break;
      case 2: //+rz sequence 1111
        shift_data = step0 + step1 + step2 + dir0 + dir1 + dir2 ;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir1 + dir2;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break;
      case 3: //stop 
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
        break; 
    }
  }

  // This case is for autoleveling function
  // This uses the direction input to determine which stabilizer is going to move
  // only one stabilizer moves at a time. There is a distinct case for each, including holding  
//  if (run_autoleveling){

    
//    if (use_stepdir){
//      
//      microseconds_per_step = 37500/max_stabilizer_rpm;
//      
//      switch (dir){
//        case 0: // move stabilizer 0 up
//          shift_data = step0 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = address_pin;
//          send_stepdir_data(); 
//          break;
//  
//        case 1: // move stabilizer 0 down
//          shift_data = step0 + dir0 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = dir0 + address_pin;
//          send_stepdir_data(); 
//          break;
//          
//        case 2: // move stabilizer 1 up
//          shift_data = step1 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = address_pin;
//          send_stepdir_data(); 
//          break;
//  
//        case 3: // move stabilizer 1 down
//          shift_data = step1 + dir1 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = dir1 + address_pin;
//          send_stepdir_data(); 
//          break;
//  
//        case 4: // move stabilizer 2 up
//          shift_data = step3 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = address_pin;
//          send_stepdir_data(); 
//          break;
//  
//        case 5: // move stabilizer 2 down
//          shift_data = step3 + dir3 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = dir3 + address_pin;
//          send_stepdir_data(); 
//          break;
//  
//        case 7: // Stop movement of all stabilizer (HOLD ALL)
//          shift_data = address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = address_pin;
//          send_stepdir_data(); 
//          break;
//          
//        case 8: // move stabilizer 0 and stabilizer 1 up
//          shift_data = step0 + step1 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = address_pin;
//          send_stepdir_data(); 
//          break;
//          
//        case 9: // move stabilizer 0 and stabilizer 1 down
//          shift_data = step0 + step1 + dir0 + dir1 + address_pin;
//          send_stepdir_data();
//          // set step low
//          shift_data = dir0 + dir1 + address_pin;
//          send_stepdir_data(); 
//          break;
//          
//        case 10:
//          send_stepdir_data();
//          shift_data = address_pin;
//          send_stepdir_data();
//          break;
//      }
//    }
//    if (use_uart){
      
//      vactual_speed = 22500;
//      
//      switch (dir){
//        case 0: // move stabilizer 0 up
//          stab_driver0.shaft(!shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(vactual_speed);
//          stab_driver1.VACTUAL(0);
//          stab_driver2.VACTUAL(0); 
//          Serial.print(" S0 up");
//          break;
//  
//        case 1: // move stabilizer 0 down
//          stab_driver0.shaft(shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(vactual_speed);
//          stab_driver1.VACTUAL(0);
//          stab_driver2.VACTUAL(0); 
//          break;
//          
//        case 2: // move stabilizer 1 up
//          stab_driver1.shaft(!shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(0);
//          stab_driver1.VACTUAL(vactual_speed);
//          stab_driver2.VACTUAL(0);
//          break;
//  
//        case 3: // move stabilizer 1 down
//          stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(0);
//          stab_driver1.VACTUAL(vactual_speed);
//          stab_driver2.VACTUAL(0);
//          break;
//  
//        case 4: // move stabilizer 2 up
//          stab_driver2.shaft(!shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(0);
//          stab_driver1.VACTUAL(0);
//          stab_driver2.VACTUAL(vactual_speed);
//          break;
//  
//        case 5: // move stabilizer 2 down
//          stab_driver2.shaft(shaft_direction);  // shaft is set by boolean
//          stab_driver0.VACTUAL(0);
//          stab_driver1.VACTUAL(0);
//          stab_driver2.VACTUAL(vactual_speed);
//          break;
//  
//        case 7: // Stop movement of all stabilizer (HOLD ALL)
//          stab_driver0.VACTUAL(0);
//          stab_driver1.VACTUAL(0);
//          stab_driver2.VACTUAL(0); 
//          break;
//          
//        case 8: // move stabilizer 0 and stabilizer 1 up
//          stab_driver0.shaft(!shaft_direction); 
//          stab_driver1.shaft(!shaft_direction); 
//          stab_driver0.VACTUAL(vactual_speed);
//          stab_driver1.VACTUAL(vactual_speed);
//          stab_driver2.VACTUAL(0);
//          break;
//          
//        case 9: // move stabilizer 0 and stabilizer 1 down
//          stab_driver0.shaft(shaft_direction); 
//          stab_driver1.shaft(shaft_direction); 
//          stab_driver0.VACTUAL(vactual_speed);
//          stab_driver1.VACTUAL(vactual_speed);
//          stab_driver2.VACTUAL(0); 
//          break;
//          
//        case 10:
//          send_stepdir_data();
//          shift_data = address_pin;
//          send_stepdir_data();
//          break;
//      }
//    }
//  }
}  


// automatic leveling 
// will raise frame automatically until the acceleromter reads level
// function removes manual control from the joysticks 
// This function currently uses the step direction interface
void autoleveling(){
  Serial.print(" AUTOLEVELING ");
  // auto leveling variables 
  joy_norm = 0.5; // Set dedicated
  
  // Set flag saying vehicle is not level
  bool vehicle_level = false;
  
  // first home the stabilizers to get a good step count 
  // First check if stabilizers have been homed. IF not run this homing protocol 
  if (!all_stabilizers_home) {home_stabilizers();}
  
  // check to see if stabilizers are tocuhing the ground
  contact_ground();

  // once stabilizer feet are all touching the ground, raise the vehicle one full rotation
  // all stabilizers are raised at once  
  lift_from_ground();
  
  // now perform auto leveling 
  while (!vehicle_level){
    Wire.beginTransmission(accel_register);
    Wire.write(0x3B);  
    Wire.endTransmission(false);
    Wire.requestFrom(accel_register,12,true);  
    accel_x_input = Wire.read()<<8|Wire.read();
    accel_y_input = Wire.read()<<8|Wire.read();

    // Determine which side is low or skip if currently leveling 
     
    if(!is_leveling) {find_low_side(); is_leveling = true;}

    if(is_leveling)  {level_side(low_side);}

    // Stop leveling flags
    if (abs(accel_x_input) < end_leveling_value) {is_leveling = false;}                       
    if (abs(accel_y_input) < end_leveling_value) {is_leveling = false;} 
    if (abs(accel_x_input) < end_leveling_value & abs(accel_y_input) < end_leveling_value) {vehicle_level = true;}
    
  }
}


// function homes stabilizers one at atime, starting with stabilizer 0
// function compares the SG_RESULT() with a defined stall gaurd value
// if the SG_RESULT() is less than the the defined stall value, a stall is detected 
// This is repeated for each stabilizer 
// This function uses while loops, so there is no manual control 
void home_stabilizers(){
  Serial.print("GO HOME");
  vactual_speed = 25000;
  // for loop runs through all stabilizers
  for (int index = 0; index <= 2; index++){
    // switch chages which stabilizer is controlled 
    switch(index){
      // stabilizer 0 
      case 0:
        // while loop continues to move stabilizer until a stall is detected 
        while(!stabilizer0_home){
            stab_driver0.shaft(shaft_direction);  // shaft is set by boolean
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0); 
            if(stab_driver0.SG_RESULT() > detect_stall_value) {stabilizer0_home = true;}
            //stabilizer_move(1);
            Serial.print("Moving S0 | SGR = ");Serial.print(stab_driver0.SG_RESULT());
            Serial.println();
        }
        break;
      // stabilizer 1  
      case 1:
        while(!stabilizer1_home){
            stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0); 
            if(stab_driver1.SG_RESULT() > detect_stall_value) {stabilizer1_home = true;}
            //stabilizer_move(3);
            Serial.print("Moving S1 | SGR = ");Serial.print(stab_driver1.SG_RESULT());
            Serial.println();
        } 
        break;
      // stabilizer 2 
      case 2:
        while(!stabilizer2_home){
            stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed);
            if(stab_driver2.SG_RESULT() > detect_stall_value) {stabilizer2_home = true;}
            stabilizer_move(4);
            Serial.print("Moving S2 | SGR = ");Serial.print(stab_driver2.SG_RESULT());
            Serial.println();
        }
        break; 
    }
  }
  all_stabilizers_home = true; // set the flag for all stabilizers home
  all_stabilizers_down = false; // reset stabilizer down flag incase it doesnt reset
}

void contact_ground(){
  stabilizer0_step_count = 0;
  stabilizer1_step_count = 0;
  stabilizer2_step_count = 0;
  // for loop runs through all stabilizers
  for (int index = 0; index <= 2; index++){
      // switch chages which stabilizer is controlled 
      switch(index){
        // stabilizer 0 
        case 0:
          // while loop continues to move stabilizer until a stall is detected 
          while(!stabilizer0_home){
              if(stab_driver0.SG_RESULT() - detect_stall_value < 0) {stabilizer0_down = true; break;}
              stabilizer_move(1);
              stabilizer0_step_count++;
          }
        // stabilizer 1  
        case 1:
          while(!stabilizer1_home){
              if(stab_driver1.SG_RESULT() - detect_stall_value < 0) {stabilizer1_down = true; break;}
              stabilizer_move(3);
              stabilizer1_step_count++;
          } 
        // stabilizer 2 
        case 2:
          while(!stabilizer2_home){
              if(stab_driver2.SG_RESULT() - detect_stall_value < 0) {stabilizer2_down = true; break;}
              stabilizer_move(5);
              stabilizer2_step_count++;
          } 
      }
   }
   all_stabilizers_home = false;
   all_stabilizers_down = true; // set the flag for all stabilizers home
}

// Logic to determine lowest side 
// The auto leveling function finds lowest side, raises until side is level then repeats 
void find_low_side(){
  //
  if (abs(accel_x_input) > abs(accel_y_input)){
      if ((accel_x_input) < 0) {low_side = 1;} // -x is low
      else                     {low_side = 2;} // +x is low
    }
  else{
      if ((accel_y_input) < 0) {low_side = 3;} // -y is low
      else                     {low_side = 4;} // +y is low
  } 
}

// raises the lowest side, determined by logic in function
// increments position counter on lowest side 
void level_side(int low_side){
  switch (low_side){
    case 1:
      stabilizer0_step_count++; // Increment step counter 
      stabilizer_move(1);       // pass to stabilizer move function
      break;
    case 2:
      stabilizer1_step_count++;
      stabilizer_move(3);
      break;
    case 3:
      stabilizer2_step_count++;
      stabilizer_move(5);
      break; 
    case 4:
      stabilizer0_step_count++;
      stabilizer1_step_count++;
      stabilizer_move(8);
      break;
  }
}

// lower all stabilizers a defined amount
// increment the step counters
void lift_from_ground(){
  
  use_stepdir = true;         // to raise all at once, use the casefor manual movement 
  run_autoleveling = false;  // this changes it into moving all stabilizers at once 
  
  for (int raise_loop = 0; raise_loop < raise_count; raise_loop++){

    stabilizer_move(1); // call stability movement function   
    
    stabilizer0_step_count++; // increment stabilizer step counts 
    stabilizer1_step_count++;
    stabilizer2_step_count++;
  }

  use_stepdir = false;      // set back into individual movement mode
  run_autoleveling = true;
}

// return all stabilizers back home. 
// THis function lowers raises all stabilizers simulataneously 
// Functions looks at step counts of stabilizers 
void return_home(){
  // Find which stabilizer moved the most, this value sets a for loop 
  long most_steps = max(stabilizer0_step_count,max(stabilizer1_step_count,stabilizer2_step_count));

  for (int return_step_loop = 0; return_step_loop < most_steps; return_step_loop++){
    // reset data sent to shift registers 
    int shift0_data = 0;  // stabilizer 0 data
    int shift1_data = 0;  // stabilizer 1 data 
    int shift2_data = 0;  // stabilizer 2 data
    
    // Add stabilizer value to total if they have not returned home 
    if (stabilizer0_step_count < return_step_loop) {shift0_data = step0;}
    if (stabilizer1_step_count < return_step_loop) {shift1_data = step1;}
    if (stabilizer2_step_count < return_step_loop) {shift2_data = step2;}

    // Sum up shift data 
    shift_data = shift0_data + shift1_data + shift2_data + address_pin;
    stabilizer_move(10); 
  }
}
