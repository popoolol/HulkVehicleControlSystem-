// Following functions change the state of system_bool 
//
// system_bool is one byte containing true false states for general system control 
// bit number          7       6        5         4         3        2           1       0
// system_bool = [PRINTING, MOVING] [ATTACHED, DETACHED] [MANUAL, AUTOMATIC] [WHEEL, STABILIZER] 

// Flag for determining if currently printing 
void set_printing_flag()
{
    if (!bitRead(system_bool, 7)) {system_bool += B10000000;}
    if  (bitRead(system_bool, 6)) {system_bool -=  B1000000;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.print("PRINTING        ");
}

// Flag for determining if moving 
void set_moving_flag()
{
    if  (bitRead(system_bool, 7)) {system_bool -= B10000000;} 
    if (!bitRead(system_bool, 6)) {system_bool +=  B1000000;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.print("MOVING PRINTER  ");
}

// Flag determing if printer is currenly attached (Can likely remove later)
// Set attached flag true, detached flag false 
void set_attached_flag()
{
    if (!bitRead(system_bool, 5)) {system_bool +=   B100000;}
    if  (bitRead(system_bool, 4)) {system_bool -=    B10000;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.print("ATTACHED        ");
}

// Flag if printer is detached from vehicle
// This was created due to the difficulty of load sensing the stabilizers
// Skips load sensing operations if printer is detached
// Set detached flag true, attached flag false 
void set_detached_flag()
{
    if  (bitRead(system_bool, 5)) {system_bool -=   B100000;}
    if (!bitRead(system_bool, 4)) {system_bool +=    B10000;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.print("DETACHED        ");
}

// Flag if performing manual actions, joystick movement control 
// Set manual flag true, automatic flag false 
void set_manual_flag()
{
    if (!bitRead(system_bool, 3)) {system_bool +=     B1000;} 
    if  (bitRead(system_bool, 2)) {system_bool -=      B100;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.print("MANUAL: ");
}

// Flag if running an automatic function, leveling, homing, pathing 
// Set automatic flag true, manual flag false 
void set_auto_flag()
{
    if  (bitRead(system_bool, 3)) {system_bool -=     B1000;}
    if (!bitRead(system_bool, 2)) {system_bool +=      B100;} 
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.clear();
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.print("AUTO   :");
}

// Flag if controlling wheels
// Set wheel flag true, stabilizer flag false
void set_wheel_flag()
{
    if (!bitRead(system_bool, 1)) {system_bool +=       B10;} 
    if  (bitRead(system_bool, 0)) {system_bool -=        B1;}
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.setCursor(8,0);lcd.print("  WHEEL");
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.setCursor(0,1);lcd.print("FREE MOVEMENT   ");
}

// Flag if controlling stabilizers 
// Set stabilizer flag true, wheel flag false 
void set_stabilizer_flag()
{
    if  (bitRead(system_bool, 1)) {system_bool -=       B10;}
    if (!bitRead(system_bool, 0)) {system_bool +=        B1;}  
    byte push_byte = 0;
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte); 
    lcd.setCursor(8,0);lcd.print("STABLIZR");
    digitalWrite(9, LOW);
    shiftOut(9, 10, MSBFIRST, push_byte);  
    lcd.setCursor(0,1);lcd.print("CONTROL: ALL    ");
}


//auto_bool is one byte containing information about automatic control state
// bit number     7        6         5       4        3          2         1            0
// auto_bool = [LEVLING, LEVELED] [HOMING, HOMED] [GROUNDING, GROUNDED] [PATHING , DESTINATION]

// Flag if currently perfoming leveling 
// Set leveling flag true, leveled flag false 
void set_leveling_flag()
{
    if (!bitRead(auto_bool, 7)) {auto_bool += B10000000;} // Set leveling flag to leveling
    if  (bitRead(auto_bool, 6)) {auto_bool -=  B1000000;} // Set leveled flag to not level
    lcd.setCursor(0,1);lcd.print("LEVELING...     ");
}

// Flag if vehicle is level 
// Set leveled flag true, leveling flag false 
void set_leveled_flag()
{
    if  (bitRead(auto_bool, 7)) {auto_bool -= B10000000;} 
    if (!bitRead(auto_bool, 6)) {auto_bool +=  B1000000;} 
    lcd.setCursor(0,1);lcd.print("PRINTER LEVELED ");
}

// Flag if homing stabilizers 
// Set homing flag true, homed flag false
void set_homing_flag()
{
    if (!bitRead(auto_bool, 5)) {auto_bool +=   B100000;} 
    if  (bitRead(auto_bool, 4)) {auto_bool -=    B10000;}
    lcd.setCursor(0,1);lcd.print("HOMING...       ");
}

// Flag if stabilizers are successfully homed 
// Set homed flag true, homing flag false 
void set_homed_flag()
{
    if  (bitRead(auto_bool, 5)) {auto_bool -=   B100000;} 
    if (!bitRead(auto_bool, 4)) {auto_bool +=    B10000;} 
    lcd.setCursor(0,1);lcd.print("ALL HOMED       ");
}

// Flag if vehicle is currently grounding
// Set grounding flag true, grounded flag false
void set_grounding_flag()
{
    if (!bitRead(auto_bool, 3)) {auto_bool +=     B1000;}
    if  (bitRead(auto_bool, 2)) {auto_bool -=      B100;}
    digitalWrite(3,LOW);
    lcd.setCursor(0,1);lcd.print("GROUNDING...    ");
    digitalWrite(3,HIGH);
}

// Flag if vehicle is successfully grounded
// Set grounded flag true, grounding flag false
void set_grounded_flag()
{
    if  (bitRead(auto_bool, 3)) {auto_bool -=     B1000;}
    if (!bitRead(auto_bool, 2)) {auto_bool +=      B100;}
    digitalWrite(3,LOW);
    lcd.setCursor(0,1);lcd.print("ALL GROUNDED    ");
    digitalWrite(3,HIGH);
}

// Flag if vehicle is currently pathing
// Set pathing flag true, destination flag false
void set_pathing_flag()
{
    if (!bitRead(auto_bool, 1)) {auto_bool +=       B10;}
    if  (bitRead(auto_bool, 0)) {auto_bool -=        B1;}
    lcd.setCursor(0,1);lcd.print("RUNNING PATH    ");
}

// Flag if vehicle is successfully navigated path 
// Set destination flag true, pathing flag false 
void set_destination_flag()
{
    if  (bitRead(auto_bool, 1)) {auto_bool -=       B10;}
    if (!bitRead(auto_bool, 0)) {auto_bool +=        B1;}
    lcd.setCursor(0,1);lcd.print("PATH COMPLETE   ");
}
