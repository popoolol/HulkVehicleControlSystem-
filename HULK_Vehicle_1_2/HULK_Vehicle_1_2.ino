#include <TMCStepper.h>     // For stability motor driver control (TMC2209)
#include <SoftwareSerial.h> // For software serial (TMC2209)
#include <Wire.h>           // for accelerometer control (GY512)
#include <SPI.h>            // For wheel driver communication (AMIS-30543)
#include <AMIS30543.h>      // For wheel motor control 

// SOFTWARE SERIAL PINS, DEFINITIONS //
// Define UART Pins 
// Trying to save memory by eliminating extra variables.
// const byte stab_tx_pin  = 9;  // UART transmission pin Eliminating this pin
const byte stab_rx_pin  = 7;  // UART receiving pin
const byte stab_tx_pin  = 6;  // UART receiving pin

// Enabling Software Serial Connection
SoftwareSerial stab_soft_serial(stab_rx_pin, stab_tx_pin);     // Be sure to connect RX to TX with resistor. Driver has one pin

AMIS30543 stepper;

//// TMC2209 Stabilizer Driver Hardware information
//                         //(SERIAL PORT, SENSE RESISTOR VALUE, DRIVER ADDRESS)
TMC2209Stepper stab_driver0(&stab_soft_serial, 0.11f, B00);   // stability driver 0
TMC2209Stepper stab_driver1(&stab_soft_serial, 0.11f, B01);   // stability driver 1
TMC2209Stepper stab_driver2(&stab_soft_serial, 0.11f, B10);   // stability driver 2


// ANALOG DEVICES //
// Joystick variables 
int    joy0_x_input, joy0_y_input;  // Joystick 0 variables (global because shared between functions)
int    joy1_x_input, joy1_y_input;  // Joystick 1 variables 
int8_t joy0_x_shift, joy0_y_shift;  // Joysticks may not read in the center, this will determine an offset at starup
int8_t joy1_x_shift, joy1_y_shift;

unsigned long last_move_time = 0;   // Debouncer for joystick control

byte system_bool = 0;    // Start booleans off as zero
byte stabilizer_bool = 0;

byte sg_raw;
const byte sg_average_count = 3;
byte sg_running_average[sg_average_count];

const byte accel_average_count = 5;
int accel_x_running_average[accel_average_count];
int accel_y_running_average[accel_average_count];

int accel_x_input, accel_y_input; // first 16 bits of register is x data, next 16 is y data
int accel_x_shift = 57;
int accel_y_shift = -20;

byte now_controlling;
byte low_side;
bool leveling = false;
bool x_level = false;
bool y_level = false;

// Boolean for currently printing 
bool printing;

unsigned long loop_counter = 0;
byte sg_delay;



//== Setup ===============================================================================
void setup() 
{
    // SERIAL CONNECITON RATES //
    // BEGIN SERIAL COMMUNICATIONS //
    Serial.begin(112500);  // initialize hardware serial for debugging
    setup_stabilizers();
    setup_wheels();
    
    Serial.println();
    Serial.println();
    Serial.print("Established Serial Connections");
    Serial.println();
    delay(200);
  
    // Choose starting system to control 
    //set_wheel_flag();
    set_manual_flag();
    set_stabilizer_flag(); // start with stabilizers 
    set_attached_flag();
    set_moving_flag();
    
    Serial.print("Enabled System Control");
    Serial.println();
    delay(200);

    setup_accel();
    zero_joysticks();
    // zero_accel();
    
    Serial.print("Retrieved Analog Device Offsets");
    Serial.println();
    delay(200);
  
    Serial.print("SETUP COMPLETE!");
    Serial.println();
    Serial.println();
    delay(1000);
}

//== MAIN LOOP ===============================================================================

void loop() 
{
    // Check if printing 
    if (!printing)
    {    
        // Read joystick values 
        read_joysticks();

        // Process joystick inputs
        determine_move(); 

        read_accel();
        accel_smoother();
    }

    // Check if in automatic control 
    if (bitRead(system_bool,4)) 
    {
        // Check if currently leveling
        if (bitRead(stabilizer_bool,7)) 
        {
            // Check if legs are grounded and if printer is attached 
            if (!bitRead(stabilizer_bool, 1) & bitRead(system_bool, 7)) 
            {
                // Run gounding protocol to contact legs with ground
                ground_stabilizers();
            }
            // Run autoleveling only if all legs are grounded 
            else
            {
                // Read current accelerometer values 
                read_accel();
                // Smooth accelerometer values
                accel_smoother();
                // Process accelerometer values 
                run_autoleveling();
            }
        }
        
        // Check if homing stabilizers 
        if (bitRead(stabilizer_bool, 6)) 
        { 
            home_stabilizers();
        }
    }
    ++loop_counter;
    DEBUG_PRINT(); 
    //delay(200);
}
