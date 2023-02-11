// function homes stabilizers one at atime, starting with stabilizer 0
// function compares the SG_RESULT() with a defined stall gaurd value
// if the SG_RESULT() is less than the the defined stall value, a stall is detected 
// This is repeated for each stabilizer 
// This function uses while loops, so there is no manual control 
//void home_stabilizers(){
//  Serial.print("GO HOME");
//  int detect_stall_value = 250; // This is what SG_RESULT is compared to, a lower number needs more torque to indicate a stall [0..255]
//  vactual_speed = 80000;
//  // for loop runs through all stabilizers
//  for (int index = 0; index <= 2; index++){
//    // switch chages which stabilizer is controlled 
//    switch(index){
//      // stabilizer 0 
//      case 0:
//        // while loop continues to move stabilizer until a stall is detected 
//        while(!stabilizer0_home){
//            stab_driver0.shaft(shaft_direction);  // shaft is set by boolean
//            stab_driver0.VACTUAL(vactual_speed);
//            stab_driver1.VACTUAL(0);
//            stab_driver2.VACTUAL(0); 
//            if(stab_driver0.SG_RESULT() > detect_stall_value) {stabilizer0_home = true;}
//            //stabilizer_move(1);
//            Serial.print("Moving S0 | sg_return = ");Serial.print(stab_driver0.SG_RESULT());
//            Serial.println();
//        }
//        break;
//      // stabilizer 1  
//      case 1:
//        while(!stabilizer1_home){
//            stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
//            stab_driver0.VACTUAL(0);
//            stab_driver1.VACTUAL(vactual_speed);
//            stab_driver2.VACTUAL(0); 
//            if(stab_driver1.SG_RESULT() > detect_stall_value) {stabilizer1_home = true;}
//            //stabilizer_move(3);
//            Serial.print("Moving S1 | sg_return = ");Serial.print(stab_driver1.SG_RESULT());
//            Serial.println();
//        } 
//        break;
//      // stabilizer 2 
//      case 2:
//        while(!stabilizer2_home){
//            stab_driver1.shaft(shaft_direction);  // shaft is set by boolean
//            stab_driver0.VACTUAL(0);
//            stab_driver1.VACTUAL(0);
//            stab_driver2.VACTUAL(vactual_speed);
//            if(stab_driver2.SG_RESULT() > detect_stall_value) {stabilizer2_home = true;}
//            stabilizer_move(4);
//            Serial.print("Moving S2 | sg_return = ");Serial.print(stab_driver2.SG_RESULT());
//            Serial.println();
//        }
//        break; 
//    }
//  }
//  all_stabilizers_home = true; // set the flag for all stabilizers home
//  all_stabilizers_down = false; // reset stabilizer down flag incase it doesnt reset
//}

void contact_ground(){
  bool stabilizer0_down = false;
  bool stabilizer1_down = false;
  bool stabilizer2_down = false;
  
  unsigned long stabilizer_start_ground_time;
  
  int lower_stabilizer = 2;
  
  byte stabilizer0_stall_value = 212;
  byte stabilizer1_stall_value = 225;
  byte stabilizer2_stall_value = 217;
 
  vactual_speed = 80000;
  Serial.println();
  Serial.print("Grounding Stabilizers....");
  Serial.println();
  
  // for loop runs through all stabilizers
  for (int index = 0; index <= 2; index++){
      // switch chages which stabilizer is controlled 
      switch(index){
        // stabilizer 0 
        case 0:
          // Set loop variables  
          loop_counter = 0;
          Serial.print("Grounding S0...");Serial.println();
          stabilizer_start_ground_time = millis();
          
          // while loop continues to move stabilizer until a stall is detected 
          while(!stabilizer0_down){
              stabilizer_move(2 + 2*index + lower_stabilizer);
              sg_smoother();
              if(sg_smooth - stabilizer0_stall_value < 0 & loop_counter > 6) {
                  stabilizer0_move_time = millis() - stabilizer_start_ground_time;
                  stabilizer0_down = true; 
              }
              loop_counter++;
              Serial.print(sg_smooth);
              Serial.println();
          }
          Serial.print("S0 grounded: Time = ");Serial.print(stabilizer0_move_time);Serial.println();
          delay(200);
          break;
        // stabilizer 1  
        case 1:
          // Reset loop variables  
          loop_counter = 0;
          Serial.print("Grounding S1...");Serial.println();
          stabilizer_start_ground_time = millis();
          // while loop continues to move stabilizer until a stall is detected 
          while(!stabilizer1_down){
              stabilizer_move(2 + 2*index + lower_stabilizer);
              sg_smoother();
              if(sg_smooth - stabilizer1_stall_value < 0 & loop_counter > 6) {
                  stabilizer1_move_time = millis() - stabilizer_start_ground_time;
                  stabilizer1_down = true; 
              }
              loop_counter++;
              Serial.print(sg_smooth);
              Serial.println();
          }
          Serial.print("S1 grounded: Time = ");Serial.print(stabilizer1_move_time);Serial.println();
          delay(200);
          break;
        // stabilizer 2 
        case 2:
          loop_counter = 0;
          Serial.print("Grounding S2...");Serial.println();
          stabilizer_start_ground_time = millis();
          // while loop continues to move stabilizer until a stall is detected 
          while(!stabilizer2_down){
              stabilizer_move(2 + 2*index + lower_stabilizer);
              sg_smoother();
              if(sg_smooth - stabilizer2_stall_value < 0 & loop_counter > 6) {
                  stabilizer2_move_time = millis() - stabilizer_start_ground_time;
                  stabilizer2_down = true; 
              }
              loop_counter++;
              Serial.print(sg_smooth);
              Serial.println();
          }
          Serial.print("S2 grounded: Time = ");Serial.print(stabilizer2_move_time);Serial.println();
          delay(200);
          
          break;
      }
   }
   all_stabilizers_home = false;
   all_stabilizers_down = true; // set the flag for all stabilizers home
}



// return all stabilizers back home. 
// THis function lowers raises all stabilizers simulataneously 
// Functions looks at step counts of stabilizers 
//void return_home(){
//  // Find which stabilizer moved the most, this value sets a for loop 
//  long most_steps = max(stabilizer0_step_count,max(stabilizer1_step_count,stabilizer2_step_count));
//
//  for (int return_step_loop = 0; return_step_loop < most_steps; return_step_loop++){
//    // reset data sent to shift registers 
//    int shift0_data = 0;  // stabilizer 0 data
//    int shift1_data = 0;  // stabilizer 1 data 
//    int shift2_data = 0;  // stabilizer 2 data
//    
//    // Add stabilizer value to total if they have not returned home 
//    if (stabilizer0_step_count < return_step_loop) {shift0_data = step0;}
//    if (stabilizer1_step_count < return_step_loop) {shift1_data = step1;}
//    if (stabilizer2_step_count < return_step_loop) {shift2_data = step2;}
//
//    // Sum up shift data 
//    shift_data = shift0_data + shift1_data + shift2_data + address_pin;
//    stabilizer_move(10); 
//  }
//}

void sg_smoother(){

  byte sg_next_index = loop_counter % sg_average_count;

  if (sg_return == 0){
    sg_return = 220;
  }
  
  sg_running_average[sg_next_index] = sg_return;
  sg_smooth = 0;

  for(int i=0; i< sg_average_count; ++i)
  {
    sg_smooth += sg_running_average[i];
  }
  sg_smooth /= sg_average_count;
}
    
 
