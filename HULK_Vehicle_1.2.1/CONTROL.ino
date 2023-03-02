// ============================= MANUAL CONTROL ===========================
// This function is the main decision tree for the joystick inputs (manual control)
// Decides what to do with joystick inputs 
// main branch swaps between wheel and stabilizer control
// non-movement inputs (autoleveling, groungin, pathing, homing) must be held to activate 
void determine_move()
{
    const byte deadzone = 150;
    const uint16_t hold_count = 200;
    
    // Analog joystick logic 
    // Check to see if one joystick is outside the deadzone
    if (abs(joy0_x_input) > deadzone | abs(joy0_y_input) > deadzone | abs(joy1_x_input) > deadzone | abs(joy1_y_input) > deadzone)
    {
       // MANUAL MODE process joystick input as direction
       if (bitRead(system_bool, 3))
       {   
            // Logic controlled by if statement tree
            // Logic only considers one joystick movement at a time 
            
            // Move vehicle forward | Lift all / single stabilizer
            if(joy0_x_input > deadzone &  abs(joy0_x_input) > abs(joy0_y_input))       // Left joystick up 
            {   
                // Simply pass to movement functions 
                if (bitRead(system_bool, 1)) {wheel_move(1);}         
                if (bitRead(system_bool, 0)) {stabilizer_move(1 + 2*now_controlling);}
                input_counter = 0; // Reset this counter         
            }
            
            // Move vehicle backward | lower all / single stabilizer
            else if (joy0_x_input < -deadzone &  abs(joy0_x_input) > abs(joy0_y_input))// Left Stick down
            {   
                // Pass to movement functions 
                if (bitRead(system_bool,1)) {wheel_move(2);}            
                if (bitRead(system_bool,0)) {stabilizer_move(2 + 2*now_controlling);}
                input_counter = 0; // Reset this counter
            }

            // Move vehicle right | ground stabilizers
            else if (joy0_y_input > deadzone &  abs(joy0_y_input) > abs(joy0_x_input)) // Left stick right 
            {   
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(3);      // Pass to movement function
                    input_counter = 0;  // Reset input counter 
                }
                
                // STABILIZER MODE 
                // Non movement inputs must be debounced
                // Input must only be in desired direction             
                if (bitRead(system_bool, 0) & abs(joy0_x_input) < deadzone) 
                {
                    // Ground stabilizer input, to debounce must hold for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles
                    else 
                    {
                        Serial.println("Grounding Stabilizers...");
                        set_auto_flag();                    // Enable automatic control 
                        set_grounding_flag();               // Flag to start and continue grounding
                        now_controlling = 0;                // Reset stabilizer being controlled 
                        input_counter = 0;                  // Reset input counter after successful input
                        tone(2, 2093);
                        delay(250);
                        noTone(2);
                    }
                } 
            }
            
            // Move vehicle left | home stabilizers
            else if (joy0_y_input < -deadzone &  abs(joy0_y_input) > abs(joy0_x_input)) // Left stick left 
            {
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(4);      // Pass to movement function
                    input_counter = 0;  // Reset input counter 
                }   
                         
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x_input) < deadzone) 
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        Serial.println("Homing Stabilizers...");
                        set_auto_flag();                        // Enable automatic control 
                        set_homing_flag();                      // Set flag to start or continue homing
                        now_controlling = 0;                    // Reset stabilizer being controlled 
                        input_counter = 0;                      // Reset input counter after successful input
                        tone(2, 2349);
                        delay(250);
                        noTone(2);
                    } 
                } 
            }
            
            // Change between stabilizers and wheel control  
            else if(joy1_x_input > deadzone &  abs(joy1_x_input) > abs(joy1_y_input))  // Right stick up
            {         
                // Check for wheel control  
                if (bitRead(system_bool, 1) & abs(joy0_y_input) < deadzone) 
                {
                    // Only allow inputs after direction is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {    
                        Serial.println("Changed to stabilizers...");
                        wheel_move(0);         // First make sure the wheels are not moving 
                        set_stabilizer_flag(); // Set flag for controlling stabilizers 
                        input_counter = 0;     // Reset input counter
                        tone(2, 2349);
                        delay(250);
                        noTone(2);                  
                    }
                }   

                // Check for stabilizer control 
                else if (bitRead(system_bool, 0) & abs(joy0_y_input) < deadzone) 
                {    
                    // Only allow inputs after direction is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {    
                        Serial.println("Changed to Wheels...");
                        stabilizer_move(0); // First make sure the stabilizers are not moving 
                        set_wheel_flag();   // Set flag for controlling stabilizers  
                        input_counter = 0;  // Reset input counter
                        tone(2, 2637);
                        delay(250);
                        noTone(2);                  
                    }
                }    
            }
            
            // Run path | Autolevel vehicle 
            else if (joy1_x_input < -deadzone &  abs(joy1_x_input) > abs(joy1_y_input))  // Right stick down 
            {   
                // WHEEL CONTROL     
                if (bitRead(system_bool, 1) & abs(joy0_y_input) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < 10) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        Serial.println("Running Path...");
                        set_auto_flag();                    // Remove manual control 
                        set_pathing_flag();                 // Set flag to begin or continue pathing 
                        current_step = 0;                   // Reset path step count (should make it so this does not always reset)
                        path_counter = 0;                   // Reset the current path number (Should change this as well)
                        input_counter = 0;                  // Reset input counter after successful input
                        tone(2, 2794);
                        delay(250);
                        noTone(2);
                    }  
                }
                
                // STABILIZER CONTROL 
                else if (bitRead(system_bool, 0) & abs(joy0_y_input) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        Serial.println("Autoleveling...");
                        set_auto_flag();                        // Set in automatic control mode 
                        set_leveling_flag();                    // Set flag to start or continue leveling
                        found_low = false;                      // Reset leveling boolean for determining low side 
                        input_counter = 0;                      // Reset input counter after successful input
                        tone(2, 3136);
                        delay(250);
                        noTone(2);
                    }
                }    
            }
            
            // move vehicle clockwise | change to next stabilizer 
            else if (joy1_y_input > deadzone &  abs(joy1_y_input) > abs(joy1_x_input))  // Right stick right 
            {   
                // WHEEL MODE  
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(5);     // Pass to wheel movement function
                    input_counter = 0; // Reset input counter 
                }            
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x_input) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        Serial.println("Next Stabilizer... Now Controlling: ");
                        now_controlling = (now_controlling + 5) % 4; // Switch to next stabilizer
                        input_counter = 0;                           // Reset input counter after successful input
                        Serial.println(now_controlling);
                        if (now_controlling == 0)
                        {
                            tone(2, 2349);
                            delay(500);
                            noTone(2);
                        }
                        else
                        {
                            for (byte idx = 0; idx < now_controlling; ++idx)
                            {
                                tone(2, 2349);
                                delay(100);
                                noTone(2);
                            }
                        }
                    }
                }
            }

            // move vehicle counterclockwise | change to previous stabilizer 
            else if (joy1_y_input < - deadzone &  abs(joy1_y_input) > abs(joy1_x_input))  // Right stick left 
            {   
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(6);     // Pass to wheel movement function 
                    input_counter = 0; // Reset input counter 
                }            
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x_input) < deadzone)
                {
                    
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        Serial.print("Previous Stabilizer...Now Controlling: ");
                        now_controlling = (now_controlling + 3) % 4; // Switch to previous stabilizer
                        input_counter = 0;                           // Reset input counter after successful input
                        Serial.println(now_controlling);
                        if (now_controlling == 0)
                        {
                            tone(2, 2349);
                            delay(500);
                            noTone(2);
                            delay(100);
                        }
                        else
                        {
                            for (byte idx = 0; idx < now_controlling; ++idx)
                            {
                                tone(2, 2349);
                                delay(100);
                                noTone(2);
                                delay(100);
                            }
                        }
                    }
                }
            }
        }

        // AUTOMATIC MODE 
        // When an input is registered during automatic control, it is assumed something is wrong
        // The current process is stopped, and manual control is initiated 
        else 
        {
            if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
            else 
            {
                auto_bool = 0;     // Reset stabilizer flag, bad because all stabilizer information is lost 
                set_manual_flag(); // Return to manual mode 
                Serial.println("Returned to Manual");
                input_counter = 0;
                tone(2, 3520);
                delay(500);
                noTone(2);
                
            }
        }
    }

    // JOYSTICK STILL IN DEADZONE 
    else 
    {
        // MANUAL MODE 
        if (bitRead(system_bool, 3))
        {
            // Stop movement 
            if (bitRead(system_bool, 1)) {wheel_move(0);}            
            if (bitRead(system_bool, 0)) {stabilizer_move(0);}
        }

        input_counter = 0; // Reset move counter 
    } 
}

// ============================= AUTOMATIC CONTROL ===========================

// function homes stabilizers one at atime, starting with stabilizer 0
// function compares the SG_RESULT() with a defined stall gaurd value
// if the SG_RESULT() is less than the the defined stall value, a stall is detected 
// This is repeated for each stabilizer 
// This function uses while loops, so there is no manual control 
void home_stabilizers(){
    // Declare the homing limit switch pin
    pinMode(5, INPUT);

    // Switch between stabilizers, only one controlled at a time  
    switch(now_controlling)
    {
        case 0:
            // Raise stabilizer 0
            stabilizer_move(1);
            
            // Read & check state of button, Button is high when pressed
            if (digitalRead(5)) 
            {
                // This means the button is pressed
                // Lower stabilizer to unpress button
                stabilizer_move(2);
                delay(100);         // Delay ensures the button is unpressed 

                // Stop moving stabilizers
                stabilizer_move(0);

                // Tell user what's happened 
                Serial.println("First Contact...");

                // Increment controlling to switch stabilizer
                ++now_controlling;
            }
            break;
        case 1:
            // Raise stabilizer 0
            stabilizer_move(3);
            
            // Read & check state of button, Button is high when pressed
            if (digitalRead(5)) 
            {
                // This means the button is pressed
                // Lower stabilizer to unpress button
                stabilizer_move(4);
                delay(100);         // Delay ensures the button is unpressed 

                // Stop moving stabilizers
                stabilizer_move(0);

                // Tell user what's happened 
                Serial.println("Stab 0 homed");

                // Increment controlling to switch stabilizer
                ++now_controlling;
            }
            break;

        case 3:
            // Raise stabilizer 1
            stabilizer_move(5);
            if (digitalRead(5)) 
            {
                stabilizer_move(6);
                delay(100);
                stabilizer_move(0);
                Serial.println("Stab 1 homed");
                ++now_controlling;
            }
            break;

        case 2:
            // Raise stabilizer 2
            stabilizer_move(7);
            if (digitalRead(5)) 
            {
                stabilizer_move(8);
                delay(100);
                stabilizer_move(0);
                Serial.println("Stab 2 homed");
                
                // Final case returns to manual control 
                now_controlling = 0;
                set_manual_flag();
                set_homed_flag();
                Serial.println("Returning to Manual Control...");
                delay(500);
            }
            break;
            
    }
}

// automatic leveling 
// will raise frame automatically until the acceleromter reads level
void run_autoleveling()
{
    
    // Max positive or negative value for reading a level condition
    byte end_value = 5;         
    
    // Determine which side is low
    if (!found_low)
    {
        // Check which side is low 
        if (abs(accel_x_input) > abs(accel_y_input))
        {
            if (accel_x_input > 0) {low_side = 1;} // Determine what low means
            else                   {low_side = 0;} // 
        }
        else
        {
            if (accel_y_input > 0) {low_side = 3;} //
            else                   {low_side = 2;} // 
        }
        // keep leveling until low side is level
        found_low = true;
    }
    
    // Raise side determined by logic
    switch(low_side)
    {
        Serial.println("Raising...");
        case 0: stabilizer_move(4); break;
        case 1: stabilizer_move(6);  break;
        case 2: stabilizer_move(8);  break;
        case 3: stabilizer_move(10);  break;
    }

//    // If low side is brought to level, find new low side 
    if (abs(accel_x_input) < end_value) 
    {
        if (low_side == 1 | low_side == 0)
        {
            stabilizer_move(0);
            delay(300);
            found_low = false;
        }
        Serial.println("found low");
    }
    if (abs(accel_y_input) < end_value)
    {
        if (low_side == 2 | low_side == 3)
        {
            stabilizer_move(0);
            delay(300);
            found_low = false;
        }
        Serial.println("Found Low");
    }
    
    // If all sides are level, end leveling
    if (abs(accel_x_input) < end_value & abs(accel_y_input) < end_value)
    {
        stabilizer_move(0);
        set_manual_flag();
        set_leveled_flag();
        set_grounded_flag();
        now_controlling = 0;
        Serial.println(" Autoleveling Complete....");
        Serial.print(accel_x_input);Serial.print(" ");Serial.println(accel_y_input);
        delay(1000);
    }
}

void ground_stabilizers(){

    //byte sg_raw;
    byte sg_smooth = 255;
    
    byte stabilizer_stall_value = 215;
    
    switch (now_controlling)
    {
        case 0:
            ++sg_delay;
            sg_raw = stabilizer_move(4);
            sg_smooth = sg_smoother(sg_raw);
            if (sg_smooth -  stabilizer_stall_value < 0 &  sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling++;
                Serial.print("S0 GROUNDED ");
                delay(200);
                sg_delay = 0;
            }
            //Serial.print(sg_raw);Serial.print(" ");Serial.print(sg_smooth); 
            break;  

        case 1:
            ++sg_delay;
            sg_raw = stabilizer_move(6);
            sg_smooth = sg_smoother(sg_raw);
            if (sg_smooth -  stabilizer_stall_value < 0 & sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling++;
                Serial.print("S1 GROUNDED ");
                delay(200);
                sg_delay = 0;
            }
            //Serial.print(sg_smooth); 
            break;

        case 2:
            ++sg_delay;
            sg_raw = stabilizer_move(8);
            sg_smooth = sg_smoother(sg_raw);
            if (sg_smooth -  stabilizer_stall_value < 0 & sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling = 0;
                Serial.println("S2 GROUNDED ");
                delay(1000);
                stabilizer_move(2);
                delay(300);
                stabilizer_move(0);
                delay(2000);
                set_grounded_flag();
                sg_delay = 0;
            }
            //Serial.print(sg_smooth); 
            break;
    }
}

// Running average 
// Note, maybe combine running average functions? 
byte sg_smoother(byte sg_raw){
    
    byte sg_next_index = loop_counter % sg_average_count;
  
    if (sg_raw == 0) {sg_raw = 255;}
    
    sg_running_average[sg_next_index] = sg_raw;
    
    byte sg_smooth = 0;
  
    for(int i=0; i< sg_average_count; ++i){sg_smooth += sg_running_average[i] / sg_average_count;}

    Serial.println(sg_smooth);
    
    return sg_smooth;
}


// Determines number of steps required to move in a certain direction 
// Distance is in mm  

void run_path(uint16_t distance, byte path_direction, byte num_microstep)
{
        // Direction correction
    float transverse_mult = 1.18;
    float rotational_mult = 1.25;
    
    byte wheel_diameter = 96; // milimeters 
    byte step_res = 200;      // step resolution of motors 

    // Calculate steps per mm
    float steps_per_mm = step_res * num_microstep / (3.14159 * wheel_diameter);
    
    // Determine total steps
    if (path_direction == 3 | path_direction == 4)
    {
        total_step_count = steps_per_mm * distance * transverse_mult;
    }
    else if (path_direction == 5 | path_direction == 6)
    {
        total_step_count = steps_per_mm * distance * rotational_mult;
    }
    else
    {
        total_step_count = steps_per_mm * distance;
    }
    if (bitRead(auto_bool, 1))
    {
        // When running path, increment current step count
        ++current_step;
        // Move vehicle in desired direction
        wheel_move(path_direction);
    }
    if (current_step > total_step_count)
    {
        set_destination_flag();
    }
    Serial.println(current_step);
}

// function for running multi-step paths
void complex_path(byte num_paths, uint16_t distance_array[], byte direction_array[])
{  
    Serial.print(path_counter);
    if (bitRead(auto_bool, 1))
    {
        run_path(distance_array[path_counter], direction_array[path_counter], 4);
    }

    if (bitRead(auto_bool, 0))
    {
        ++path_counter;
        Serial.print("Path ");Serial.print(path_counter);Serial.println(" Completed");
        current_step = 0;
        
        if (path_counter < num_paths)
        {
            set_pathing_flag();
        }
        else
        {
            set_manual_flag();
            Serial.print("Pathing Complete!");
        }
    }
}
