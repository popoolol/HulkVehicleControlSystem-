// Move Stabilizer handles all stabilizer movement, manual or automatic
// * UART and Step-direction 
// * Controlling all at once, controlling individual 
// Broken into two main parts, uart and stepdir
// Both parts have the same number of cases, and the cases are the same for the two protocols 
// The vactual speed, or microstep_per_second is set in the function that calls move_stabilizer
byte stabilizer_move(byte dir){       

    // Default stallGaurd value, when not asked to pull it 
    //byte sg_raw = 255; 

    //uint32_t vactual_speed = max(stab_driver0.VACTUAL(),max(stab_driver1.VACTUAL(),stab_driver2.VACTUAL()));
     
    // The stabilizer movement speed is constant. There is no real need to have a variable speed
    uint32_t vactual_speed = 80000; // This speed is about 1 rotation per second 
    bool shaft_direction;           // true turns motors forward, false turns motors backward. 
        
    switch(dir)// Switch between priority directions
    {   
        // The cases set the motor direction with shaft(), then sets the speed with VACTUAL()
        // Cases 0 - 2 mave all stabilizers simulataneously 
        // Cases 3 - 8 move stabilizers individually 
        // Cases 9 -10 move stabilizers 0 and 1 together (for leveling)
        case 0: // all stop 
        
            vactual_speed = 0;
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            Serial.print(" ALL STOP ");
            break; 
            
        case 1: // All Up
            
            stab_driver0.shaft(!shaft_direction); // shaft sets motor direction
            stab_driver1.shaft(!shaft_direction); // !shaft_direction reverses direction
            stab_driver2.shaft(!shaft_direction); 
               
            stab_driver0.VACTUAL(vactual_speed);  // Set speed 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            Serial.print(" All Up ");
            break;
            
        case 2: // All down
        
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);  
            stab_driver2.shaft(shaft_direction);     
            
            stab_driver0.VACTUAL(vactual_speed); 
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(vactual_speed);
            Serial.print(" All Down ");
            break;    
              
        case 3: // move stabilizer 0 up
            // Cases 3 - 8 also retrive the result of the stall gaurd for sensorless homing 
            stab_driver0.shaft(!shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);  // SG_RESULT() gets the stallgaurd value
            sg_raw = stab_driver0.SG_RESULT(); // This value is called IMMEDIATELY after movement for best results 
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0); 
            Serial.print(" S0 Up ");
            break; 
        case 4: // move stabilizer 0 down
        
            stab_driver0.shaft(shaft_direction); 
             
            stab_driver0.VACTUAL(vactual_speed);
            sg_raw = stab_driver0.SG_RESULT();
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(0);
            Serial.print(" S0 Down ");
            break;  
                 
        case 5: // move stabilizer 1 up
        
            stab_driver1.shaft(!shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            Serial.print(" S1 Up ");
            break;
            
        case 6: // move stabilizer 1 down
        
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(vactual_speed);
            sg_raw = stab_driver1.SG_RESULT();
            stab_driver2.VACTUAL(0);
            Serial.print(" S1 Down ");
            break;
            
        case 7: // move stabilizer 2 up
        
            stab_driver2.shaft(!shaft_direction); 
             
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed / 2);
            sg_raw = stab_driver2.SG_RESULT();
            Serial.print(" S2 Up ");
            break;
        case 8: // move stabilizer 2 down
            stab_driver2.shaft(shaft_direction);  
            
            stab_driver0.VACTUAL(0);
            stab_driver1.VACTUAL(0);
            stab_driver2.VACTUAL(vactual_speed / 2);
            sg_raw = stab_driver2.SG_RESULT();
            Serial.print(" S2 Down ");
            break;       
        case 9: // move stabilizer 0 and stabilizer 1 up
        
            stab_driver0.shaft(!shaft_direction); 
            stab_driver1.shaft(!shaft_direction); 
            
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0);
            Serial.print(" S0 & S1 up");
            break;        
        case 10: // move stabilizer 0 and stabilizer 1 down
            stab_driver0.shaft(shaft_direction); 
            stab_driver1.shaft(shaft_direction);
             
            stab_driver0.VACTUAL(vactual_speed);
            stab_driver1.VACTUAL(vactual_speed);
            stab_driver2.VACTUAL(0); 
            Serial.print(" S0 & S1 down");
            break;
    }
    return sg_raw;
} 

uint32_t stabilizer_acc(uint32_t current_speed, uint32_t desired_speed)
{
    // Number of steps desired to reach speed
    byte desired_steps = 10;

    uint32_t acceleration_step = desired_speed / desired_steps;
    if (desired_speed != 0)
    {
        if      (current_speed < acceleration_step  ) {current_speed = acceleration_step;  }
        else if (current_speed < acceleration_step*2) {current_speed = acceleration_step*2;}
        else if (current_speed < acceleration_step*3) {current_speed = acceleration_step*3;}
        else if (current_speed < acceleration_step*4) {current_speed = acceleration_step*4;}
        else if (current_speed < acceleration_step*5) {current_speed = acceleration_step*5;}
        else if (current_speed < acceleration_step*6) {current_speed = acceleration_step*6;}
        else if (current_speed < acceleration_step*7) {current_speed = acceleration_step*7;}
        else if (current_speed < acceleration_step*8) {current_speed = acceleration_step*8;}
        else if (current_speed < acceleration_step*9) {current_speed = acceleration_step*9;}
        else                                          {current_speed = desired_speed;      }
    }
    if (desired_speed == 0)
    {
        if      (current_speed > acceleration_step*9) {current_speed = acceleration_step*9;}
        else if (current_speed > acceleration_step*8) {current_speed = acceleration_step*8;}
        else if (current_speed > acceleration_step*7) {current_speed = acceleration_step*7;}
        else if (current_speed > acceleration_step*6) {current_speed = acceleration_step*6;}
        else if (current_speed > acceleration_step*5) {current_speed = acceleration_step*5;}
        else if (current_speed > acceleration_step*4) {current_speed = acceleration_step*4;}
        else if (current_speed > acceleration_step*3) {current_speed = acceleration_step*3;}
        else if (current_speed > acceleration_step*2) {current_speed = acceleration_step*2;}
        else if (current_speed > acceleration_step*1) {current_speed = acceleration_step;  }
        else                                          {current_speed = desired_speed;      }
    }

    return current_speed;
}
