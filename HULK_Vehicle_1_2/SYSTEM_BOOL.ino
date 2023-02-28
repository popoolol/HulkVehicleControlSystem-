// Following functions change the state of system_bool 
// system_bool is one byte containing true false states for general system control 
// system_bool reference of flag location
//    bit number        7         6          5         4         3        2             1       0
// system_bool     = [ATTACHED, DETACHED] [MANUAL, AUTOMATIC] [WHEEL, STABILIZER] [PRINTING, MOVING]
// stabilizer_bool =  [LEVLING, HOMING]    [LEVELED, HOMED]  [GROUNDING,        ]    [GROUNDED,   ]
// For example: manually controlling one stabilizer with UART, system_bool = 10100101
//              Autoleveling individually controls stabilizers with UART, system_bool = 10010101
//              Typical driving controls all wheels manually, if using step-dir, system_bool = 01101010
// THESE CAN LIKELY BE MADE INTO SINGLE FUNCTION WITH SWITCH STATEMENT

void set_attached_flag()
{
    if (!bitRead(system_bool, 7)) {system_bool += B10000000;}
    if  (bitRead(system_bool, 6)) {system_bool -=  B1000000;}
}

void set_dettached_flag()
{
    if  (bitRead(system_bool, 7)) {system_bool -= B10000000;}
    if (!bitRead(system_bool, 6)) {system_bool +=  B1000000;}
}

// Enable the wheel drivers for manual control, 
// Will also disable the stabilizer drivers 
void set_wheel_flag()
{
    // Enable wheel drivers, LOW = active.  
    // Addresses are hardwired and set by jumper 
    if (!bitRead(system_bool, 3)) {system_bool += B1000;} // if wheel flag was false, set it true 
    // Stabilizer drivers enabled/ disabled by setting output 6 on the stabilizer shift register 01000000
    // set this output high to disable these drivers 
    if  (bitRead(system_bool, 2)) {system_bool -=  B100;}   
}

// Enable the stabilizer drivers for manaul control 
// Will also disable the wheel drivers 
void set_stabilizer_flag()
{
    // Enable stabilizers drivers, LOW = active.  
    // Stabilizer enable pin is controlled by 6th bit sent to shift register B01000000 
    if (!bitRead(system_bool, 2)) {system_bool +=  B100;} 
    if  (bitRead(system_bool, 3)) {system_bool -= B1000;} // Check wheel flag, set bit 0 if flag was true 
}

// Enable automatic control
// This currently removes all joystick inputs
// May be a little redundant since the auto leveling typically runs in while loop for better polling rates  
void set_auto_flag()
{
    if (!bitRead(system_bool, 4)) {system_bool +=    B10000;} // if auto flag was false, set it true 
    if (bitRead(system_bool, 5) ) {system_bool -=   B100000;} // if manual flag was true, set it false
}
void set_manual_flag()
{
    if  (bitRead(system_bool, 7)) {system_bool -= B10000000;}
    if  (bitRead(system_bool, 6)) {system_bool -=  B1000000;}
    if (!bitRead(system_bool, 5)) {system_bool +=   B100000;} // if auto flag was false, set it true 
    if  (bitRead(system_bool, 4)) {system_bool -=    B10000;} // if manual flag was true, set it false
}

void set_printing_flag()
{
    if (!bitRead(system_bool, 1)) {system_bool +=       B10;}
    if  (bitRead(system_bool, 0)) {system_bool -=        B1;}
}

void set_moving_flag()
{
    if  (bitRead(system_bool, 1)) {system_bool -=       B10;} 
    if (!bitRead(system_bool, 0)) {system_bool +=        B1;} 
}



void set_leveling_flag()
{
    if (!bitRead(stabilizer_bool, 7)) {stabilizer_bool += B10000000;} // Set leveling flag to leveling
    if  (bitRead(stabilizer_bool, 5)) {stabilizer_bool -=   B100000;} // Set leveled flag to not level
}

void set_leveled_flag()
{
    if  (bitRead(stabilizer_bool, 7)) {stabilizer_bool -= B10000000;} // Set leveling flag to not leveling
    if (!bitRead(stabilizer_bool, 5)) {stabilizer_bool +=   B100000;} // Set leveled flag to level
}

void set_homing_flag()
{
    if (!bitRead(stabilizer_bool, 6)) {stabilizer_bool +=  B1000000;} // Set leveling flag to leveling
    if  (bitRead(stabilizer_bool, 4)) {stabilizer_bool -=    B10000;} // Set leveled flag to not level
}

void set_homed_flag()
{
    if  (bitRead(stabilizer_bool, 6)) {stabilizer_bool -=  B1000000;} // Set leveling flag to not leveling
    if (!bitRead(stabilizer_bool, 4)) {stabilizer_bool +=    B10000;} // Set leveled flag to level
}

void set_grounding_flag()
{
    if (!bitRead(stabilizer_bool, 3)) {stabilizer_bool +=     B1000;} // Set grounding flag to grounding
    if  (bitRead(stabilizer_bool, 1)) {stabilizer_bool -=       B10;} // Set grounded flag to not grounded 
}

void set_grounded_flag()
{
    if  (bitRead(stabilizer_bool, 3)) {stabilizer_bool -=     B1000;} // Set grounding flag to not grounding
    if (!bitRead(stabilizer_bool, 1)) {stabilizer_bool +=       B10;} // Set grounded flag to grounded 
}
