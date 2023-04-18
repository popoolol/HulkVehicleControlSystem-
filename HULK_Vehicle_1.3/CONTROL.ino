// ============================= MANUAL CONTROL ===========================
/*
 * determine_move() is used to process joystick inputs
 * Joystick values
 *    Values are analog natively be between 0-1024
 *    To get negative values the joystick inputs are immediately remapped between -+511
 *    A shift is applied to the joysticks
 *        The joysticks do not always read (0,0) at a neutral position, the shift remaps the neutral position to (0,0)
 * Variables         
 *    deadzone   - the minimum joystick movement required to read an action
 *    hold_count - number of loop cycles required to register an input
 *                 User must hold the joystick in a direction for the desired loop cycles to register input 
 *                 functions as a debouncer, since the joystick inputs can sometimes be erratic 
 * Logic                 
 *    Check if any joystick is moved outside of deadzone 
 *    Check if manual mode or automatic mode 
 *    Manual Mode 
 *        Left joystick is prioritized
 *            This joystick is checked first, if an input is read the move is determined 
 *            Can not perform simultaneous inputs
 *                If there is an input on the lft and right joystick, only the left joystick will be read 
 *             
 */
void determine_move()
{
    // Begin by polling the joysticks for input
    int joy0_x = map(analogRead(A0), 0, 1024, 511, -511) - joy0_x_shift; // Read x direction, remap to have negative values
    int joy0_y = map(analogRead(A1), 0, 1024, 511, -511) - joy0_y_shift; // Read y direction, remap to have nagative values 
    int joy1_x = map(analogRead(A2), 0, 1024, 511, -511) - joy1_x_shift;                
    int joy1_y = map(analogRead(A3), 0, 1024, 511, -511) - joy1_y_shift;

 /*   
  *  Reference for system_bool bit readings
  *  byte manual_mode      = 3;
  *  byte auto_mode        = 2;
  *  byte wheel_mode       = 1;
  *  byte stabilizer_mode  = 0; 
  */
    
    const byte deadzone = 150;
    const uint16_t hold_count = 200;
    
    // Analog joystick logic 
    // Check to see if one joystick is outside the deadzone
    if (abs(joy0_x) > deadzone | abs(joy0_y) > deadzone | abs(joy1_x) > deadzone | abs(joy1_y) > deadzone)
    {
       // MANUAL MODE process joystick input as direction
       if (bitRead(system_bool, 3))
       {   
            // ==== Left joystick up ======
            // Move vehicle forward | Lift all / single stabilizer
            if(joy0_x > deadzone &  abs(joy0_x) > abs(joy0_y))
            {   
                input_counter = 0; // Reset the counter  
                // Simply pass to movement functions 
                if (bitRead(system_bool, 1)) {wheel_move(1);  return;                        }         
                else                         {stabilizer_move(1 + 2*now_controlling); return;}       
            }
            
            // Move vehicle backward | lower all / single stabilizer
            else if (joy0_x < -deadzone &  abs(joy0_x) > abs(joy0_y))// Left Stick down
            {   
                input_counter = 0;
                // Pass to movement functions 
                if (bitRead(system_bool,1)) {wheel_move(2); return;                         }            
                if (bitRead(system_bool,0)) {stabilizer_move(2 + 2*now_controlling); return;} 
            }

            // Move vehicle right | ground stabilizers
            else if (joy0_y > deadzone &  abs(joy0_y) > abs(joy0_x)) // Left stick right 
            {   
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(3);      // Pass to movement function
                    input_counter = 0;  // Reset input counter
                    return; 
                }
                
                // STABILIZER MODE 
                // Non movement inputs must be debounced
                // Input must only be in desired direction             
                if (bitRead(system_bool, 0) & abs(joy0_x) < deadzone) 
                {
                    // Ground stabilizer input, to debounce must hold for 10 cycles 
                    if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles
                    else 
                    {
                        set_auto_flag();                    // Enable automatic control 
                        set_grounding_flag();               // Flag to start and continue grounding
                        now_controlling = 0;                // Reset stabilizer being controlled 
                        input_counter = 0;                  // Reset input counter after successful input
                        tone(5, 2093);
                        delay(250);
                        noTone(5);
                        return;
                    }
                } 
            }
            
            // Move vehicle left | home stabilizers
            else if (joy0_y < -deadzone &  abs(joy0_y) > abs(joy0_x)) // Left stick left 
            {
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(4);      // Pass to movement function
                    input_counter = 0;  // Reset input counter 
                    return;
                }   
                         
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x) < deadzone) 
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles 
                    else 
                    {
                        set_auto_flag();                        // Enable automatic control 
                        set_homing_flag();                      // Set flag to start or continue homing
                        now_controlling = 0;                    // Reset stabilizer being controlled 
                        input_counter = 0;                      // Reset input counter after successful input
                        tone(5, 2349);
                        delay(250);
                        noTone(5);
                        return;
                    } 
                } 
            }
            
            // Change between stabilizers and wheel control  
            else if(joy1_x > deadzone &  abs(joy1_x) > abs(joy1_y))  // Right stick up
            {         
                // Check for wheel control  
                if (bitRead(system_bool, 1) & abs(joy0_y) < deadzone) 
                {
                    // Only allow inputs after direction is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles 
                    else 
                    {    
                        wheel_move(0);         // First make sure the wheels are not moving 
                        set_stabilizer_flag(); // Set flag for controlling stabilizers 
                        input_counter = 0;     // Reset input counter
                        tone(5, 2349);
                        delay(250);
                        noTone(5);   
                        return;               
                    }
                }   

                // Check for stabilizer control 
                else if (bitRead(system_bool, 0) & abs(joy0_y) < deadzone) 
                {    
                    // Only allow inputs after direction is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles 
                    else 
                    {    
                        stabilizer_move(0); // First make sure the stabilizers are not moving 
                        set_wheel_flag();   // Set flag for controlling stabilizers  
                        input_counter = 0;  // Reset input counter
                        tone(5, 2637);
                        delay(250);
                        noTone(5); 
                        return;                 
                    }
                }    
            }
            
            // Run path | Autolevel vehicle 
            else if (joy1_x < -deadzone &  abs(joy1_x) > abs(joy1_y))  // Right stick down 
            {   
                // WHEEL CONTROL     
                if (bitRead(system_bool, 1) & abs(joy0_y) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < 10) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        set_auto_flag();                    // Remove manual control 
                        set_pathing_flag();                 // Set flag to begin or continue pathing 
                        current_step = 0;                   // Reset path step count (should make it so this does not always reset)
                        path_counter = 0;                   // Reset the current path number (Should change this as well)
                        input_counter = 0;                  // Reset input counter after successful input
                        tone(5, 2794);
                        delay(250);
                        noTone(5);
                        return;
                    }  
                }
                
                // STABILIZER CONTROL 
                else if (bitRead(system_bool, 0) & abs(joy0_y) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        set_auto_flag();                        // Set in automatic control mode 
                        set_leveling_flag();                    // Set flag to start or continue leveling
                        found_low = false;                      // Reset leveling boolean for determining low side 
                        input_counter = 0;                      // Reset input counter after successful input
                        tone(5, 3136);
                        delay(250);
                        noTone(5);
                        return;
                    }
                }    
            }
            
            // move vehicle clockwise | change to next stabilizer 
            else if (joy1_y > deadzone &  abs(joy1_y) > abs(joy1_x))  // Right stick right 
            {   
                // WHEEL MODE  
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(5);     // Pass to wheel movement function
                    input_counter = 0; // Reset input counter 
                    return;
                }            
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x) < deadzone)
                {
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter;}
                    // After 10 cycles 
                    else 
                    {
                        lcd.setCursor(9,1);
                        now_controlling = (now_controlling + 5) % 4; // Switch to next stabilizer
                        input_counter = 0;                           // Reset input counter after successful input
                        if (now_controlling == 0)
                        {
                            lcd.print("ALL    ");
                            tone(5, 2349);
                            delay(500);
                            noTone(5);
                            return;
                        }
                        else
                        {
                            lcd.print(now_controlling);lcd.print("      ");
                            for (byte idx = 0; idx < now_controlling; ++idx)
                            {
                                tone(5, 2349);
                                delay(100);
                                noTone(5);
                                delay(100);
                            }
                            return;
                        }
                    }
                }
            }

            // move vehicle counterclockwise | change to previous stabilizer 
            else if (joy1_y < - deadzone &  abs(joy1_y) > abs(joy1_x))  // Right stick left 
            {   
                // WHEEL MODE 
                if (bitRead(system_bool, 1)) 
                {
                    wheel_move(6);     // Pass to wheel movement function 
                    input_counter = 0; // Reset input counter 
                    return;
                }            
                // STABILIZER MODE 
                if (bitRead(system_bool, 0) & abs(joy0_x) < deadzone)
                {
                    
                    // Make sure the input is held for 10 cycles 
                    if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles 
                    else 
                    {
                        lcd.setCursor(9,1);
                        now_controlling = (now_controlling + 3) % 4; // Switch to previous stabilizer
                        input_counter = 0;                           // Reset input counter after successful input
                        if (now_controlling == 0)
                        {
                            lcd.print("ALL    ");
                            tone(5, 2349);
                            delay(250);
                            noTone(5);
                            return;  
                        }
                        else
                        {
                            lcd.print(now_controlling);lcd.print("      ");
                            for (byte idx = 0; idx < now_controlling; ++idx)
                            {
                                tone(5, 2349);
                                delay(100);
                                noTone(5);
                                delay(100);
                            }
                            return;
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
            if (input_counter < hold_count) {++input_counter; return;}
                    // After 10 cycles 
            else 
            {
                auto_bool = 0;     // Reset stabilizer flag, bad because all stabilizer information is lost 
                set_manual_flag(); // Return to manual mode 
                lcd.setCursor(0,1);lcd.print("PROCESS STOPPED ");
                input_counter = 0;
                tone(5, 3520);
                delay(300);
                noTone(5);
                return; 
            }
        }
    }

    // JOYSTICK STILL IN DEADZONE 
    else 
    {
        // MANUAL MODE 
        if (bitRead(system_bool, 3))
        {
             input_counter = 0; // Reset move counter
            // Stop movement 
            if (bitRead(system_bool, 1)) {wheel_move(0); return;     }            
            if (bitRead(system_bool, 0)) {stabilizer_move(0); return;}
        }
    } 
}

// ============================= AUTOMATIC CONTROL ===========================

// function homes stabilizers one at atime, starting with stabilizer 0
// function compares the SG_RESULT() with a defined stall gaurd value
// if the SG_RESULT() is less than the the defined stall value, a stall is detected 
// This is repeated for each stabilizer 
// This function uses while loops, so there is no manual control 
void home_stabilizers(){

    // Switch between stabilizers, only one controlled at a time  
    switch(now_controlling)
    {
        case 0:
            // Raise All
            stabilizer_move(1);
            
            // Button state now controlled with interrupt
            if (bitRead(system_bool, 5)) 
            {
                // Increment controlling to switch stabilizer
                stabilizer_move(2);
                delay(300);
                stabilizer_move(0);
                set_delimited_flag();
                ++now_controlling;
            }
            break;
        case 1:
            // Raise stabilizer 0
            stabilizer_move(3);
            
            if (bitRead(system_bool, 5)) 
            {
                stabilizer_move(4);
                delay(300);
                stabilizer_move(0);
                set_delimited_flag();
                ++now_controlling;
            }
            break;

        case 2:
            // Raise stabilizer 1
            stabilizer_move(5);
            
            if (bitRead(system_bool, 5)) 
            {
                stabilizer_move(6);
                delay(300);
                stabilizer_move(0);
                set_delimited_flag();
                ++now_controlling;
            }
            break;

        case 3:
            // Raise stabilizer 2
            stabilizer_move(7);
            
            if (bitRead(system_bool, 5)) 
            {
                // Final case returns to manual control 
                stabilizer_move(8);
                delay(300);
                stabilizer_move(0);
                now_controlling = 0;
                set_delimited_flag();
                set_homed_flag();
                delay(250);
                set_manual_flag();
                set_wheel_flag();
                tone(5, 2349);
                delay(50);
                noTone(5);
                delay(50);
                tone(5, 3136);
                delay(50);
                noTone(5);
                delay(50);
            }
            break;  
    }
}

// automatic leveling 
// will raise frame automatically until the acceleromter reads level
// Functions brought inside autoleveling
void run_autoleveling()
{      
    // Accelerometer reading now done entirely within function
    Wire.beginTransmission(0x68);   // Establish path for accelerometer data
    Wire.write(0x3B);               // Selecting register storing accelerometer data
    Wire.endTransmission(false);    // Continue transmission
    Wire.requestFrom(0x68,4,true);  // Retrieving 4 bytes from established path, true means stop retrieving 
  
    // Accelerometer x and y data are 16 bit values starting with most significant digit. X data is stored first 
    // 4 bytes are requested from the Gy 512, the first 2 bytes are x data and the second 2 bytes are y data
    // read function reads 8 bits of data at a time. 8 bits are read, those bits are shifted 8 places, then the next 8 bits are read
    int accel_x = (Wire.read()<<8|Wire.read()) - 75; // read byte, shift byte 8 places, read second byte. 2 bytes of x data   
    int accel_y = (Wire.read()<<8|Wire.read()) + 8; // 2 bytes of y data

    // Smoothing now done entirely within function
    // generate array index position
    byte index = loop_counter % accel_average_count;
    
    // place accelerometer x and y values into array cells
    accel_x_running_average[index] = accel_x;
    accel_y_running_average[index] = accel_y;

    // Reset smoothed value
    long int accel_x_sum = 0;
    long int accel_y_sum = 0;

    // Sum all array elements 
    for(int idx = 0; idx < accel_average_count; ++idx)
    {
      accel_x_sum += accel_x_running_average[idx];
      accel_y_sum += accel_y_running_average[idx];
    }

    // Get running average by dividing array sum by amount of array elements 
    accel_x = accel_x_sum / accel_average_count;
    accel_y = accel_y_sum / accel_average_count;        
    
    // Determine which side is low
    if (!found_low)
    {
        // Check which side is low 
        if (abs(accel_x) > abs(accel_y))
        {
            if (accel_x > 0) {low_side = 1;} // Determine what low means
            else             {low_side = 0;} // 
        }
        else
        {
            if (accel_y > 0) {low_side = 3;} //
            else             {low_side = 2;} // 
        }
        // keep leveling until low side is level
        found_low = true;
    }
    
    // Raise side determined by logic
    switch(low_side)
    {
        case 0: stabilizer_move(4); break;
        case 1: stabilizer_move(6); break;
        case 2: stabilizer_move(8); break;
        case 3: stabilizer_move(10); break;
    }

    // Max positive or negative value for reading a level condition
    byte end_value = 10; 
    
    // If low side is brought to level, find new low side 
    if (abs(accel_x) < end_value) 
    {
        if (low_side == 1 | low_side == 0)
        {
            stabilizer_move(0);
            delay(300);
            found_low = false;
        }
    }
    if (abs(accel_y) < end_value)
    {
        if (low_side == 2 | low_side == 3)
        {
            stabilizer_move(0);
            delay(300);
            found_low = false;
        }
    }
    
    // If all sides are level, end leveling
    if (abs(accel_x) < end_value & abs(accel_y) < end_value)
    {
        stabilizer_move(0);
        delay(50);
        for (int idx = 0; idx < accel_average_count; ++idx)
        {
            Wire.beginTransmission(0x68);   // Establish path for accelerometer data
            Wire.write(0x3B);               // Selecting register storing accelerometer data
            Wire.endTransmission(false);    // Continue transmission
            Wire.requestFrom(0x68,4,true);  // Retrieving 4 bytes from established path, true means stop retrieving 
          
            // Accelerometer x and y data are 16 bit values starting with most significant digit. X data is stored first 
            // 4 bytes are requested from the Gy 512, the first 2 bytes are x data and the second 2 bytes are y data
            // read function reads 8 bits of data at a time. 8 bits are read, those bits are shifted 8 places, then the next 8 bits are read
            accel_x = (Wire.read()<<8|Wire.read()) - 75; // read byte, shift byte 8 places, read second byte. 2 bytes of x data   
            accel_y = (Wire.read()<<8|Wire.read()) + 8; // 2 bytes of y data
        
            // generate array index position
            index = loop_counter % accel_average_count;
            
            // place accelerometer x and y values into array cells
            accel_x_running_average[index] = accel_x;
            accel_y_running_average[index] = accel_y;
        
            // Reset smoothed value
            accel_x_sum = 0;
            accel_y_sum = 0;
        
            // Sum all array elements 
            for(int idx = 0; idx < accel_average_count; ++idx)
            {
              accel_x_sum += accel_x_running_average[idx];
              accel_y_sum += accel_y_running_average[idx];
            }
        
            // Get running average by dividing array sum by amount of array elements 
            accel_x = accel_x_sum / accel_average_count;
            accel_y = accel_y_sum / accel_average_count;

            ++loop_counter;
        }
        if (abs(accel_x) < end_value & abs(accel_y) < end_value)
        {
            set_leveled_flag();
            digitalWrite(1, LOW);
            delay(250);
            digitalWrite(1, HIGH);
            set_printing_flag();
//            set_manual_flag();
//            set_stabilizer_flag();
//            now_controlling = 0;
        }
    }
}

void ground_stabilizers()
{
    //byte sg_raw;
    byte sg_smooth = 255;
    
    byte stabilizer_stall_value = 215;
    
    switch (now_controlling)
    {
        case 0:
            ++sg_delay;
            sg_smooth = stabilizer_move(4);
            if (sg_smooth -  stabilizer_stall_value < 0 &  sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling++;
                delay(200);
                sg_delay = 0;
            }
            break;  

        case 1:
            ++sg_delay;
            sg_smooth = stabilizer_move(6);
            if (sg_smooth -  stabilizer_stall_value < 0 & sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling++;
                delay(200);
                sg_delay = 0;
            }
            break;

        case 2:
            ++sg_delay;
            sg_smooth = stabilizer_move(8);
            if (sg_smooth -  stabilizer_stall_value < 0 & sg_delay > 5)
            {
                stabilizer_move(0);
                now_controlling = 0;
                delay(500);
                stabilizer_move(2);
                delay(500);
                // When in moving mode return to manual control, otherwise do not
                // Manual mode is unwanted during axes homing at start up
                stabilizer_move(0);
                if (bitRead(system_bool, 7))
                {
                    lcd.setCursor(0,0);lcd.print("GRND: HOMING... ");
                    // Send signal to duet
                    digitalWrite(1, LOW);
                    // Wait for Duet to respond 
                    delay(10);
                    // Check state of input pin from duet 
                    // WHen still high, duet has not 
                    if (digitalRead(0))
                    {
                       // Wait again 
                       delay(50);
                       // wait again before writing wait condition
                       digitalWrite(1, HIGH);
                    }
                    else 
                    {
                        // This case means the duet received the information and is homing
                        // Wait until a high signal is read
                        digitalWrite(1, HIGH);
                    }
                                      
                }
                if (bitRead(system_bool, 6))
                {    
                    set_manual_flag(); 
                }
                set_grounded_flag();
                sg_delay = 0;
                tone(5, 3136);
                delay(50);
                noTone(5);
                delay(50);
                tone(5, 2349);
                delay(50);
                noTone(5);
                delay(50);
            }
            break;
    }
}

// Determines number of steps required to move in a certain direction 
// Distance is in mm 
void run_path(uint16_t distance, byte path_direction, byte num_microstep)
{  
    byte wheel_diameter = 96; // milimeters 
    byte step_res = 200;      // step resolution of motors 

    // Calculate steps per mm
    float steps_per_mm = step_res * num_microstep / (3.14159 * wheel_diameter);
    
    // Determine total steps
    if (path_direction == 3 | path_direction == 4)
    {
        total_step_count = steps_per_mm * distance * 1.18;
    }
    else if (path_direction == 5 | path_direction == 6)
    {
        total_step_count = distance * 1.33 * (876.0 / 1000.0);
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
}

// function for running multi-step paths
void complex_path()
{  
    byte total_paths = 10;
    
    uint16_t distances[total_paths] = {3000, 790, 1600, 1640, 1645, 775, 1600, 100, 200, 100}; //1524 mm is 5' Rotation is in radians * 1000 (785 is quarter rotation)
                                  //F  B  C  A
    byte directions[total_paths] = {1, 5, 1, 6, 1, 5, 2, 3, 4, 3};
    
    if (bitRead(auto_bool, 1))
    {
        run_path(distances[path_counter], directions[path_counter], 4);
    }

    if (bitRead(auto_bool, 0))
    {
        ++path_counter;
        current_step = 0;
        
        if (path_counter < total_paths)
        {
            set_pathing_flag();
        }
        else
        {
            lcd.setCursor(0,1);lcd.print("ALL PATHS ENDED");
            delay(300);
            set_manual_flag();
            set_stabilizer_flag();
        }
    }
}


void stabilizer_limit()
{
    // Simply sets a flag, flag is either resolved in main loop or in automatic functions
    set_limited_flag();
    return;
}


void startup_protocol()
{
    // Startup completed in three steps: ground, home, set control flags  
    // If the stabilizers are grounded, home 
    if (bitRead(auto_bool, 2))
    {
        home_stabilizers();
    }
    // If the stabilizers are not grounded, ground them to prepare for axes homing
    else if (!bitRead(auto_bool, 2))
    {
        ground_stabilizers();
    }
    // Once the stabilziers are homed the start up sequence is complete
    else if (bitRead(auto_bool, 4))
    {
        set_moving_flag();
        set_manual_flag();
        set_wheel_flag();
    }
}
