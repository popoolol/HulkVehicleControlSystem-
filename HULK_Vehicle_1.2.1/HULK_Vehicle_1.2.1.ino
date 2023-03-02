#include <TMCStepper.h>     // For stability motor driver control (TMC2209)
#include <SoftwareSerial.h> // For software serial (TMC2209)
#include <Wire.h>           // for accelerometer control (GY512)
#include <SPI.h>            // For wheel driver communication (AMIS-30543)
#include <AMIS30543.h>      // For wheel motor control 
//#include <Tone.h>           // For buzzer 

// SOFTWARE SERIAL PINS, DEFINITIONS //
// Define UART Pins 
// Trying to save memory by eliminating extra variables.
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

uint16_t input_counter = 0;   // Debouncer for joystick control

// system_bool = [PRINTING, MOVING] [ATTACHED, DETACHED] [MANUAL, AUTOMATIC] [WHEEL, STABILIZER] 
byte system_bool = B01101001;
// auto_bool = [LEVLING, LEVELED] [HOMING, HOMED] [GROUNDING, GROUNDED] [PATHING, DESTINATION]    
byte auto_bool = 0;

byte sg_raw;
const byte sg_average_count = 3;
byte sg_running_average[sg_average_count];


int accel_x_input, accel_y_input; // first 16 bits of register is x data, next 16 is y data
int accel_x_shift = 75;
int accel_y_shift = -8;

const byte accel_average_count = 5;
int accel_x_running_average[accel_average_count];
int accel_y_running_average[accel_average_count];

byte now_controlling;
byte low_side;
bool found_low = false;

unsigned long loop_counter = 0;
byte sg_delay;

uint16_t total_step_count;
uint16_t current_step;
uint16_t current_delay = 0;


byte path_counter;

//== Setup ===============================================================================
void setup() 
{
    // SERIAL CONNECITON RATES //
    // BEGIN SERIAL COMMUNICATIONS //
    Serial.begin(115200);  // initialize hardware serial for debugging
    setup_stabilizers();
    setup_wheels();
    
    Serial.println();
    Serial.println();
    Serial.print("ESTABLISHED SERIAL CONNECTIONS");
    Serial.println();
    delay(200);
    
    Serial.print("System Control : ");
    Serial.println(system_bool, BIN);
    Serial.print("Auto Control   : ");
    Serial.println(auto_bool, BIN);
    delay(200);

    setup_accel();
    setup_joysticks();
    Serial.print("SETUP ANALOG DEVICES");
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
    byte total_paths = 5;
    uint16_t distances[total_paths] = {1524, 1524, 1524, 762, 600}; //1524 mm is 5'
                                  //F  R  L  B  C
    byte directions[total_paths] = {1, 3, 4, 2, 5};

    
    // Check if printing 
    if (bitRead(system_bool, 6))
    {    
        // Read joystick values 
        read_joysticks();

        // Process joystick inputs
        determine_move(); 

        read_accel();
        accel_smoother();
        Serial.print("X-rot: ");Serial.print(accel_x_input);Serial.print(" Y-rot: ");Serial.println(accel_y_input);
        
        
    }

    // Check if in automatic control 
    if (bitRead(system_bool, 2)) 
    {
        // Check if currently leveling
        if (bitRead(auto_bool,7)) 
        {
//            // Check if legs are grounded   
//            if (!bitRead(auto_bool, 2)) 
//            {
//                set_grounding_flag();
//                ground_stabilizers();
//            }
//            // Run autoleveling only if all legs are grounded 
//            else
//            {
                // Read current accelerometer values 
                read_accel();
                accel_smoother();
                // Process accelerometer values 
                run_autoleveling();
//            }
        }
        
        // Check if homing stabilizers 
        else if (bitRead(auto_bool, 5)) 
        { 
            home_stabilizers();
        }

        else if (bitRead(auto_bool, 3))
        {
            //ground_stabilizers();

            stabilizer_move(2);
            delay(6000);
            stabilizer_move(0);
            set_grounded_flag();
            Serial.print("Grounded?...");
        }

        // Check if currently running a path 
        else if (bitRead(auto_bool, 1))
        {
            complex_path(total_paths, distances, directions);
        }
    }
    ++loop_counter;
    //DEBUG_PRINT(); 
}
