// Enable the wheel drivers for manual control 
// Will also disable the stabilizer drivers 
void enable_wheels(){
  // Enable wheel drivers, LOW = active.  
  // Addresses can ALL be high or low, they are hardwired together 
  // Stabilizer nable pin is controlled by 7th bit sent to stabilizer shift register, set high with stabilizer_en_pin = 64 
  // Must turn off stabilizer drivers first, ensure stabilizer control by setting flags  
  control_stabilizers = true;
  control_wheels = false;
  shift_data = stabilizer_en_pin; // Sets enable pins high on stabilizers    
  send_stepdir_data();// Send shift data to stability system
  // Give control back to wheels with flags 
  control_stabilizers = false;
  control_wheels = true;
  
  digitalWrite(wheel_en_pin, LOW); // Set wheel enable pin to high, disabling wheel drivers 

  manual_control = true;
}


// Wheel 7 controls the wheel movement with 7 possible directions (including no movement)
// This function is used when in manual control 
// Function can handle both UART and STEP-DIR interfacing 
void wheel_move(int dir){       
  // Check for interface type 
  if(use_uart){

    vactual_speed = joy_norm * max_vactual; // joystick input scaled to RPM (kinda sloppy)
    
    switch(dir){// Switch between priority directions
      case 1: //+ty sequence 1100 
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data();
        break;
      case 2: //-ty sequence 0011
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        wheel_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data();
        break;
      case 3: //+txsequence 1001
        // Set motor directions
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data();
        break;
      case 4: //-tx sequence 0110
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data(); 
        break;
      case 5: //-rz sequence 0000
        // Set motor directions
        wheel_driver0.shaft(!shaft_direction);  // shaft is set by boolean
        wheel_driver1.shaft(!shaft_direction);  // !shaft is opposite direction
        wheel_driver2.shaft(!shaft_direction);   
        wheel_driver3.shaft(!shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data(); 
//        Serial.print(" Clockwise ");
        break;
      case 6: //+rz sequence 1111
        wheel_driver0.shaft(shaft_direction);   // shaft is set by boolean
        wheel_driver1.shaft(shaft_direction);   // !shaft is opposite direction
        wheel_driver2.shaft(shaft_direction);   
        wheel_driver3.shaft(shaft_direction);  
        // Set vactual_speed of motor drivers 
        send_uart_data();
//        Serial.print(" Anticlockwise ");
        break;
      case 7: //stop 
        send_uart_data();
//        Serial.print(" Stop ");
        break;  
    }
  }
  if (use_stepdir){
//    Serial.print(" |STEP-DIR|");
    
    //microseconds_per_step = min_microseconds_per_step / joy_norm / 2; // delay is called twice per pulse, hence halving delay here
    microseconds_per_step = 37500/max_stabilizer_rpm;
     
    switch(dir){// Switch between priority directions
      
      case 1:   //+ty sequence 1100 
        // Set step high
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir1;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir1;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Forward ");
        break;
      case 2: //-ty sequence 0011
        shift_data = step0 + step1 + step2 + step3 + dir2 + dir3;
        send_stepdir_data();
        // set step low
        shift_data = dir2 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Backward ");
        break;
      case 3: //+txsequence 1001
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir3;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Left ");
        break;
      case 4: //-tx sequence 0110
        shift_data = step0 + step1 + step2 + step3 + dir1 + dir2;
        send_stepdir_data();
        // set step low
        shift_data = dir1 + dir2;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Right ");
        break;
      case 5: //-rz sequence 0000
        shift_data = step0 + step1 + step2 + step3;
        send_stepdir_data();
        // set step low
        shift_data = 0;
        send_stepdir_data(); 
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Clockwise ");
        break;
      case 6: //+rz sequence 1111
        shift_data = step0 + step1 + step2 + step3 + dir0 + dir1 + dir2 + dir3;
        send_stepdir_data();
        // set step low
        shift_data = dir0 + dir1 + dir2 + dir3;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Anticlockwise ");
        break;
      case 7: //stop 
        shift_data = 0;
        send_stepdir_data();
        // set step low
        shift_data = 0;
        send_stepdir_data();
        delayMicroseconds(microseconds_per_step);
//        Serial.print(" Stop ");
        break; 
    }
  }
}  

void wheel_acceleration(){


}
