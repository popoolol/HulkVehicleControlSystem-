//// Enable autoleveling sets proper flags for auto leveling procedure 
void enable_autoleveling(){
  // Remove all manual control
  manual_control = false;   // remove manual control from the system
  control_wheels = false;
  control_stabilizers = false;
  // Enable protocol 
  //enable_stepdir();
  enable_uart();
  // Set flag for autoleveling
  run_autoleveling = true; // auto leveling operation flag 
}

// automatic leveling 
// will raise frame automatically until the acceleromter reads level
// function removes manual control from the joysticks 
// This function currently uses the step direction interface
void autoleveling(){
  Serial.print(" AUTOLEVELING ");
  
  // Set flag saying vehicle is not level
  bool vehicle_level = false;
  bool is_leveling = false;
  // check to see if stabilizers are tocuhing the ground
  contact_ground();
  Serial.println(" Vehicle Legs Contacted Ground...");
  delay(1000);
  // once stabilizer feet are all touching the ground, raise the vehicle one full rotation
  // all stabilizers are raised at once  
  lift_from_ground();
  Serial.println(" WHeels Lifted from ground...");
  delay(1000);
  // now perform auto leveling 
  while (!vehicle_level){
    read_accel();
    accel_smoother();

    // Determine which side is low or skip if currently leveling 
     
    if(!is_leveling) {find_low_side(); is_leveling = true;}

    if(is_leveling)  {level_side(low_side);}

    // Stop leveling flags
    if (abs(accel_x_input) < end_leveling_value) {is_leveling = false;}                       
    if (abs(accel_y_input) < end_leveling_value) {is_leveling = false;} 
    if (abs(accel_x_input) < end_leveling_value & abs(accel_y_input) < end_leveling_value) {vehicle_level = true;}
    
  }
  Serial.print(" AUTOLEVELING COMPLETE....");
  delay(5000);
}

// Logic to determine lowest side 
// The auto leveling function finds lowest side, raises until side is level then repeats 
void find_low_side(){
  //
  if (abs(accel_x_input) > abs(accel_y_input))
  {
      if ((accel_x_input) > 0) {low_side = 0;} // -x is low
      else                     {low_side = 1;} // +x is low
   }
  else
  {
      if ((accel_y_input) > 0) {low_side = 2;} // -y is low
      else                     {low_side = 3;} // +y is low
  } 
}
//
//// raises the lowest side, determined by logic in function
//// increments position counter on lowest side 
void level_side(int low_side){
  const byte hold_stabilizers   = 0;
  const byte raise_stabilizers  = 1;
  const byte lower_stabilizers  = 2;
  
  switch (low_side)
  {
    case 0:
      stabilizer_move(2 + low_side * 2 + lower_stabilizers);       // pass to stabilizer move function
      break;
    case 1:
      stabilizer_move(2 + low_side * 2 + lower_stabilizers);
      break;
    case 2:
      stabilizer_move(2 + low_side * 2 + lower_stabilizers);
      break; 
    case 3:
      stabilizer_move(2 + low_side * 2 + lower_stabilizers);
      break;
  }
}

// lower all stabilizers a defined amount
// increment the step counters
void lift_from_ground(){
  
  bool is_leveling = true;
  
  int lift_duration = 1000;
  int lower_stabilizer = 2;

  vactual_speed = 50000;
  
  stabilizer_move(lower_stabilizer); // call stability movement function   
  delay(lift_duration);

  stabilizer0_move_time += lift_duration;
  stabilizer1_move_time += lift_duration;
  stabilizer2_move_time += lift_duration;
}

// Begins i2c protocol 
// also determines an initial offset for the accelerometer. 
// If sitting on wheels on floor, this is assumed level 
// Comment out or don't use shift values if true level is desired
void get_accel_offset(){
  
  for (int shift_count = 0; shift_count < accel_average_count; shift_count++)
  {
    accel_x_shift = 0;
    accel_y_shift = 0;
    read_accel();
    accel_smoother();
    loop_counter++;
    //Serial.println();
  }

  accel_x_shift = accel_x_input;
  accel_y_shift = accel_y_input;

  Serial.print("AccShift: ");Serial.print(accel_x_shift);Serial.print(" ");Serial.print(accel_y_shift);Serial.println();
  loop_counter = 0;
  accel_is_offset = true;
}

void read_accel(){
  int16_t accel_x_raw;     // Creating 16 bit values for reading accelerometer data
  int16_t accel_y_raw;
  
  Wire.beginTransmission(0x68);   // Establish path for accelerometer data
  Wire.write(0x3B);               // Selecting register storing accelerometer data
  Wire.endTransmission(false);    // Continue transmission
  Wire.requestFrom(0x68,4,true);  // Retrieving 4 bytes from established path, true means stop retrieving 

  // Accelerometer x and y data are 16 bit values starting with most significant digit. X data is stored first 
  // 4 bytes are requested from the Gy 512, the first 2 bytes are x data and the second 2 bytes are y data
  // read function reads 8 bits of data at a time. 8 bits are read, those bits are shifted 8 places, then the next 8 bits are read
  accel_x_raw = Wire.read()<<8|Wire.read(); // read byte, shift byte 8 places, read second byte. 2 bytes of x data   
  accel_y_raw = Wire.read()<<8|Wire.read(); // 2 bytes of y data
  
  accel_y_input = accel_y_raw - accel_y_shift; // apply optional shift 
  accel_x_input = accel_x_raw - accel_x_shift;
  
  //Serial.print(" Raw: "); Serial.print(accel_x_raw);
  //Serial.print(" "); Serial.print(accel_y_raw);
  //Serial.print(" Shifted = "); Serial.print(accel_x_input);
  //Serial.print(" "); Serial.print(accel_y_input);
}

void accel_smoother(){
    int accel_next_index = loop_counter % accel_average_count;
    
    accel_x_running_average[accel_next_index] = accel_x_input;
    
    //Serial.print(" Shifted x: ");Serial.print(accel_x_running_average[accel_next_index]);
    //Serial.print(" Pos: ");Serial.print(accel_next_index);Serial.print(" || ");

    int accel_x_smooth = 0;

    for(int i=0; i< accel_average_count; ++i)
    {
      //Serial.print(accel_x_running_average[i]);Serial.print("  | ");
      accel_x_smooth += accel_x_running_average[i];
    }
    accel_x_input = accel_x_smooth / accel_average_count;

    accel_y_running_average[accel_next_index] = accel_y_input;
    
    //Serial.print(" Shifted y : ");Serial.print(accel_y_running_average[accel_next_index]);Serial.print(" || ");
    
    int accel_y_smooth = 0;

    for(int i=0; i< accel_average_count; ++i)
    {
      //Serial.print(accel_y_running_average[i]);Serial.print("  | ");
      accel_y_smooth += accel_y_running_average[i];
    }
    accel_y_input = accel_y_smooth / accel_average_count;

    //Serial.print(" Axy : ");Serial.print(accel_x_smooth);Serial.print(" ");Serial.print(accel_y_smooth);
}
