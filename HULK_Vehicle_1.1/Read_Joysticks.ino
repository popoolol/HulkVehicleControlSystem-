void setup_joysticks(){
  // Run this in setup to declare pin outs on arduino 
  const byte joy0_b_pin = 13;   // Button pin inputs 
  const byte joy1_b_pin = 100; //////////////////////////////////////CHANGE BACK WHEN DONE TESTING!!!!!

  pinMode(joy0_b_pin, INPUT_PULLUP);  // joystick button pin
  pinMode(joy1_b_pin, INPUT_PULLUP);  
  
}

// Gets joystick inputs: analog values in x and y and the buttons
// The buttons are toggled. joystick buttons change between controlling movement and controlling stability 
// pressing both buttons at once will set into autoleveling mode 
void read_joysticks(){
  // Define joysitck pins
  const byte joy0_x_pin   = A0;  // Analog input pins(joysticks) 
  const byte joy0_y_pin   = A1;
  const byte joy1_x_pin   = A2;
  const byte joy1_y_pin   = A3;
  const byte joy0_b_pin   = 13;  // Button pin inputs 
  const byte joy1_b_pin   = 100; //CHANGE BACK WHEN DONE TESTING!!!!!
  const int max_joy_value = 512;
  
  // Begin by polling the joysticks for input
  joy0_x_input = map(analogRead(joy0_x_pin), 0, 1024, -max_joy_value, max_joy_value) - joy0_x_shift; // Read x direction, remap to have negative values
  joy0_y_input = map(analogRead(joy0_y_pin), 0, 1024, -max_joy_value, max_joy_value) - joy0_y_shift; // Read y direction, remap to have nagative values 
  joy1_x_input = map(analogRead(joy1_x_pin), 0, 1024, -max_joy_value, max_joy_value) - joy1_x_shift;                
  joy1_y_input = map(analogRead(joy1_y_pin), 0, 1024, -max_joy_value, max_joy_value) - joy1_y_shift; 
  button0_state = digitalRead(joy0_b_pin);
  button1_state = digitalRead(joy1_b_pin);
  // optional statements to exclude second button
  button1_state = true;
  //button1_state = false;

}

// Changes manual control between the step-direction and uart interfaces,
// as well as changing manual control between the stabilizers and the wheels
// Will also set engage the autoleveling function
void system_and_interface(){
  // Variables 
  const byte debounce_time = 200;  // set desired debounce limit in ms

  
  // Logic to switch between connection interace and system
  // Switching is controlled by toggling
  if (!button0_state & button1_state) 
  { // press button 0 and not button 1
    if (millis() - last_single_button_time > debounce_time)
    {
      if    (control_stabilizers){enable_wheels();     } // flip flop between controlling wheels and stability
      else                       {enable_stabilizers();}
      last_single_button_time = millis(); // reset button time
    }
  } 
  if (button0_state & !button1_state) 
  { // press button 1 and not button 0
    if (millis() - last_single_button_time > debounce_time)
    { 
      if    (use_uart){enable_stepdir();} // flip flop between control interfaces
      else            {enable_uart();   }
      last_single_button_time = millis();
    }
  }
  if (!button0_state & !button1_state) {enable_autoleveling();}  
}


// Switch between can be used for individual wheel or stabilizer control 
// created to even out the stabilizers, They were getting out of sync
void switch_between(){
  // Variables
  const byte debounce_time = 200;  // set desired debounce limit in ms
  
  // Check check which system is being controlled 
  if (millis() - last_single_button_time > debounce_time)
  {
    if (control_wheels){now_controlling = (now_controlling + 1) % 4;} // modulus of 4 since 4 wheels. Wraps now_controlling 0-3
    // When switching from wheel to stabilizer it's possible to control a stabilizer that does not exist
    // Changes the controlling index to a stabilizer that exists
    if (control_stabilizers)
    { 
      if (now_controlling == 3) {now_controlling = 2;}  
      now_controlling = (now_controlling + 1) % 3;   // modulus of 3 since 3 wheels. Wraps now_controlling 0-2
    }
    last_single_button_time = millis();
  }
}

// logic for handling which movement type is desired 
// Choices are individual or all, wheel or stabilizer
// Passing inputs to the correct movement logic function
void select_movement(){
  
  if(control_all & control_wheels)        {move_all_wheel();        }
  if(control_indiv & control_wheels)      {move_indiv_wheel();      }
  if(control_all & control_stabilizers)   {move_all_stabilizer();   }
  if(control_indiv & control_stabilizers) {move_indiv_stabilizer(); }
}


// Determines the joystick offset
// This is run in the setup once, the shift value is applied to the joysticks to zero them
void get_joystick_offset(){
  // Define joysitck pins
  const byte joy0_x_pin   = A0;  // Analog input pins(joysticks) 
  const byte joy0_y_pin   = A1;
  const byte joy1_x_pin   = A2;
  const byte joy1_y_pin   = A3;
  const byte joy0_b_pin   = 13;  // Button pin inputs 
  const byte joy1_b_pin   = 100; //CHANGE BACK WHEN DONE TESTING!!!!!
  const int max_joy_value = 512;
  
  // Setup joysticks by checking initial offsets
  joy0_x_shift = map(analogRead(joy0_x_pin), 0, 1024, -max_joy_value, max_joy_value); // Gets an initial offset for the joystick poteniometer to correct values
  joy0_y_shift = map(analogRead(joy0_y_pin), 0, 1024, -max_joy_value, max_joy_value);
  joy1_x_shift = map(analogRead(joy1_x_pin), 0, 1024, -max_joy_value, max_joy_value);
  joy1_y_shift = map(analogRead(joy1_y_pin), 0, 1024, -max_joy_value, max_joy_value);

  Serial.print("JoyShift: ");
  Serial.print(joy0_x_shift);Serial.print(" ");
  Serial.print(joy0_y_shift);Serial.print(" ");
  Serial.print(joy1_x_shift);Serial.print(" ");
  Serial.print(joy1_y_shift);Serial.println();
  
}
