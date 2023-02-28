
// Gets joystick inputs: analog values in x and y and the buttons
// The buttons are toggled. joystick buttons change between controlling movement and controlling stability 
// pressing both buttons at once will set into autoleveling mode 
void read_joysticks()
{
    // Define joysitck pins
    const byte jx_0_pin   = A0;  // Analog input pins(joysticks) 
    const byte jy_0_pin   = A1;
    const byte jx_1_pin   = A2;
    const byte jy_1_pin   = A3;
    //const byte jb_1_pin   = 2;
    const int max_j = 511;
    
    // Begin by polling the joysticks for input
    joy0_x_input = map(analogRead(jx_0_pin), 0, 1024, max_j, -max_j) - joy0_x_shift; // Read x direction, remap to have negative values
    joy0_y_input = map(analogRead(jy_0_pin), 0, 1024, max_j, -max_j) - joy0_y_shift; // Read y direction, remap to have nagative values 
    joy1_x_input = map(analogRead(jx_1_pin), 0, 1024, max_j, -max_j) - joy1_x_shift;                
    joy1_y_input = map(analogRead(jy_1_pin), 0, 1024, max_j, -max_j) - joy1_y_shift; 
    
    //jb_state = digitalRead(jb_1_pin);
}

// This function is the main decision tree for the joystick inputs (manual control)
// Decides what to do with joystick inputs 
// main branch swaps between wheel and stabilizer control, then 
void determine_move()
{
    const byte deadzone = 30;
    
    // Analog joystick logic 
    // Check to see if one joystick is out*side the deadzone
    if (abs(joy0_x_input) > deadzone | abs(joy0_y_input) > deadzone | abs(joy1_x_input) > deadzone | abs(joy1_y_input) > deadzone)
    {
       if (bitRead(system_bool,5))
       {   
            // Logic controlled by if statement tree
            // Logic only considers one joystick movement at a time 
            if(joy0_x_input > deadzone &  abs(joy0_x_input) > abs(joy0_y_input))       // Left joystick up 
            {   // Move vehicle forward | Lift all / single stabilizer
                if (bitRead(system_bool,3)) {wheel_move(1);}  // See move functions for reference           
                if (bitRead(system_bool,2)) {stabilizer_move(1 + 2*now_controlling);}        
            }
            
            // Move vehicle backward | lower all / single stabilizer
            else if (joy0_x_input < -deadzone &  abs(joy0_x_input) > abs(joy0_y_input))// Left Stick down
            {   
                if (bitRead(system_bool,3)) {wheel_move(2);}            
                if (bitRead(system_bool,2)) {stabilizer_move(2 + 2*now_controlling);} 
            }

            // Move vehicle right | ground stabilizers
            else if (joy0_y_input > deadzone &  abs(joy0_y_input) > abs(joy0_x_input)) // Left stick right 
            {   
                if (bitRead(system_bool,3)) {wheel_move(3);}            
                if (bitRead(system_bool,2) & joy0_y_input > (deadzone * 3)) 
                {
                    Serial.println("Grounding HULK...");
                    set_auto_flag();
                    set_grounding_flag();
                    now_controlling = 0;
                } 
            }
            
            // Move vehicle left | home stabilizers
            else if (joy0_y_input < -deadzone &  abs(joy0_y_input) > abs(joy0_x_input)) // Left stick left 
            {
                if (bitRead(system_bool,3)) {wheel_move(4);}            
                if (bitRead(system_bool,2) & joy0_y_input < (-deadzone * 3)) 
                {
                    Serial.println("Homing Stabilizers...");
                    set_auto_flag();
                    set_homing_flag();
                    now_controlling = 0;
                } 
            }
            
            // Change between stabilizers and wheel control  
            else if(joy1_x_input > (deadzone * 2) &  abs(joy1_x_input) > abs(joy1_y_input))  // Right stick up
            {       
                if (millis() - last_move_time > 350) // Debounce this input
                {   
                    if (bitRead(system_bool,3)) 
                    {
                        set_stabilizer_flag(); 
                        now_controlling = 0;
                    }
                    else                        
                    {
                        set_wheel_flag();
                        now_controlling = 0;                       
                    }
                    
                    last_move_time = millis();
                }
            }
            
            // Autolevel vehicle
            else if (joy1_x_input < (-deadzone * 3) &  abs(joy1_x_input) > abs(joy1_y_input))  // Right stick down 
            {   
                 if (millis() - last_move_time > 350) // Debounce this input
                {   
                    set_stabilizer_flag(); 
                    set_auto_flag();
                    set_leveling_flag();
                    set_auto_flag();
                    leveling = false;
                    last_move_time = millis();
                }
            }
            
            // move vehicle clockwise | change to next stabilizer 
            else if (joy1_y_input > deadzone &  abs(joy1_y_input) > abs(joy1_x_input))  // Right stick right 
            {   
                if (bitRead(system_bool,3)) {wheel_move(5);}            
                if (bitRead(system_bool,2) & millis() - last_move_time > 350)
                {
                    now_controlling = (now_controlling + 5) % 4;
                    
                    last_move_time = millis();
                }
            }

            // move vehicle counterclockwise | change to previous stabilizer 
            else if (joy1_y_input < - deadzone &  abs(joy1_y_input) > abs(joy1_x_input))  // Right stick left 
            {   
                if (bitRead(system_bool,3)) {wheel_move(6);}            
                if (bitRead(system_bool,2) & millis() - last_move_time > 350)
                {
                    now_controlling = (now_controlling + 3) % 4;
                    
                    last_move_time = millis();
                }
            }

            last_move_time = millis();
        }
        else 
        {
            // Return to manual mode 
            if (millis() - last_move_time > 350) 
            {
                stabilizer_bool = 0; // Reset stabilizer flag, bad because all stabilizer information is lost 
                set_manual_flag();
            }
        }
    }
    else 
    {
        if (bitRead(system_bool,5))
        {
            if (bitRead(system_bool,3)) {wheel_move(0);}            
            if (bitRead(system_bool,2)) {stabilizer_move(0);}
        }
    } 
}

// Set a zero for the joysticks
// When the joysticks are turned on they do not always read the same zero, this will correct that 
void zero_joysticks()
{  
    //const byte jb_1_pin   = 2;
    //pinMode(jb_1_pin,INPUT_PULLUP);
    
    // Reading joysticks once seems to be enough to get good shift values 
    read_joysticks();           
    joy0_x_shift = joy0_x_input;
    joy0_y_shift = joy0_y_input;
    joy1_x_shift = joy1_x_input;
    joy1_y_shift = joy1_y_input;
    
    
    // Print shifts for debugging
//    Serial.print("JoyShift: ");
//    Serial.print(joy0_x_shift);Serial.print(" ");
//    Serial.print(joy0_y_shift);Serial.print(" ");
//    Serial.print(joy1_x_shift);Serial.print(" ");
//    Serial.print(joy1_y_shift);Serial.println();
//  
}
