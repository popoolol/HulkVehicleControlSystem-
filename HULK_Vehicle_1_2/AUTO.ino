// function homes stabilizers one at atime, starting with stabilizer 0
// function compares the SG_RESULT() with a defined stall gaurd value
// if the SG_RESULT() is less than the the defined stall value, a stall is detected 
// This is repeated for each stabilizer 
// This function uses while loops, so there is no manual control 
void home_stabilizers(){
    // Declare the homing button pin
    // const byte limit_switch_pin = 5;
    pinMode(5, INPUT);

    // Check for the 
    switch(now_controlling)
    {
        case 0:
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
            
            // Debug print statement
            Serial.print(digitalRead(5));Serial.print(" ");
            break;

        case 1:
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
            Serial.print(digitalRead(5));Serial.print(" ");
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
            Serial.print(digitalRead(5));Serial.print(" ");
            break;
        
    }
}

// automatic leveling 
// will raise frame automatically until the acceleromter reads level
// function removes manual control from the joysticks 
// This function currently uses the step direction interface
void run_autoleveling()
{
    // Max positive or negative value for reading a level condition
    byte end_value = 5;         
    
    // Determine which side is low
    if (!leveling)
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
        leveling = true;
    }
    
    // Raise side determined by logic
    switch(low_side)
    {
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
            leveling = false;
        }
    }
    if (abs(accel_y_input) < end_value)
    {
        if (low_side == 2 | low_side == 3)
        {
            stabilizer_move(0);
            delay(300);
            leveling = false;
        }
    }
    
    // If all sides are level, end leveling
    if (abs(accel_x_input) < end_value & abs(accel_y_input) < end_value)
    {
        stabilizer_move(0);
        set_manual_flag();
        set_leveled_flag();
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
                Serial.println(" Leveling HULK");
                Serial.println();
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
  
    return sg_smooth;
}
