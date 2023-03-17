
#include <LiquidCrystal_74HC595.h>// I2C Control of LCD screen
#include <TMCStepper.h>           // For stability motor driver control (TMC2209)
#include <SoftwareSerial.h>       // For software serial (TMC2209)
#include <Wire.h>                 // For accelerometer control (GY512)
#include <SPI.h>                  // For wheel driver communication (AMIS-30543)
#include <AMIS30543.h>            
// For wheel motor control 

//#include <Tone.h>           // For buzzer 

// SOFTWARE SERIAL PINS, DEFINITIONS //
// Define UART Pins 
// Trying to save memory by eliminating extra variables.
const byte stab_rx_pin  = 6;  // UART receiving pin
const byte stab_tx_pin  = 7;  // UART transmitting pin 1k resistor 

// Enabling Software Serial Connection
SoftwareSerial stab_soft_serial(stab_rx_pin, stab_tx_pin); // Be sure to connect RX to TX with resistor. Driver has one pin
LiquidCrystal_74HC595 lcd(8, 10, 9, 1, 3, 4, 5, 6, 7);
AMIS30543 stepper;

//// TMC2209 Stabilizer Driver Hardware information
//                         //(SERIAL PORT, SENSE RESISTOR VALUE, DRIVER ADDRESS)
TMC2209Stepper stab_driver0(&stab_soft_serial, 0.11f, B00);   // stability driver 0
TMC2209Stepper stab_driver1(&stab_soft_serial, 0.11f, B01);   // stability driver 1
TMC2209Stepper stab_driver2(&stab_soft_serial, 0.11f, B10);   // stability driver 2

// ANALOG DEVICES //
// Joystick variables 
int8_t joy0_x_shift, joy0_y_shift;  // Joysticks may not read in the center, this will determine an offset at starup
int8_t joy1_x_shift, joy1_y_shift;

uint8_t input_counter = 0;   // Debouncer for joystick control

// system_bool = [PRINTING, MOVING] [ATTACHED, DETACHED] [MANUAL, AUTOMATIC] [WHEEL, STABILIZER] 
byte system_bool;
// auto_bool = [LEVLING, LEVELED] [HOMING, HOMED] [GROUNDING, GROUNDED] [PATHING, DESTINATION]    
byte auto_bool;

byte sg_running_average[3];


int accel_x, accel_y; // first 16 bits of register is x data, next 16 is y data

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
uint8_t last_dir;

volatile bool limit_hit = false;
volatile bool limit_check = false;

bool shaft_direction;  // false turns motors forward, true turns motors backward. 

byte path_counter;

//== Setup ===============================================================================
void setup() 
{
    // SERIAL CONNECITON RATES //
    // BEGIN SERIAL COMMUNICATIONS //
    Serial.begin(115200);  // initialize hardware serial for debugging
    lcd.begin(16,2);
    lcd.clear();
    lcd.print("Booting...");
    
    setup_stabilizers();
    setup_wheels();
    digitalWrite(3, HIGH);
    lcd.setCursor(0,1);lcd.print("Systems Setup   ");
    delay(250);
    setup_accel();
    setup_joysticks();
    
    attachInterrupt(digitalPinToInterrupt(2), stabilizer_limit, RISING);
    
    lcd.setCursor(0,1);lcd.print("Analogs Setup   ");
    delay(250);

    lcd.setCursor(0,1);lcd.print("SETUP COMPLETE! ");
    tone(5, 2000);
    delay(100);
    noTone(5);
    delay(100);
    tone(5, 2000);
    delay(100);
    noTone(5);
    delay(500);

    set_moving_flag();
    delay(100);
    set_attached_flag();
    delay(100);
    set_manual_flag();
    delay(100);
    set_stabilizer_flag();
    delay(100);
}

//== MAIN LOOP ===============================================================================

void loop() 
{   
    
    // Check if printing 
    if (bitRead(system_bool, 6))
    {    
        // Process joystick inputs
        determine_move(); 
    }

    // Check if in automatic control 
    if (bitRead(system_bool, 2)) 
    {
        // Check if currently leveling
        if (bitRead(auto_bool,7)) 
        {
            // Check if stabilizers are grounded 
            // Must first ground stabilizers before autoleveling
            if (!bitRead(auto_bool, 2))
            {
                ground_stabilizers();
            }
            else 
            { 
                // Only when grounded, run autoleveling
                run_autoleveling();
            }
        }
        
        // Process homing stabilizers 
        else if (bitRead(auto_bool, 5)) 
        { 
            home_stabilizers();
        }

        // Process grounding 
        else if (bitRead(auto_bool, 3))
        {
            ground_stabilizers();
        }

        // Process Path running  
        else if (bitRead(auto_bool, 1))
        {
            complex_path();
        }
    }

    if (limit_check)
    {
        // Immediately stop movement
        stabilizer_move(0);
        // Beep a lot 
        for (int idx = 0; idx < 5; ++idx)
        {
            tone(5, 2349);
            delay(50);
            noTone(5);
            delay(50);
        }
        
        // Check the current shaft direction and move the stabilizers in opposite direction
        switch(shaft_direction)
        {
            case true:
                stabilizer_move(2);
                delay(300);
                stabilizer_move(0);
                break;
                
            case false:
                stabilizer_move(1);
                delay(300);
                stabilizer_move(0);
                break;
        }  
        limit_hit = false;
        limit_check = false;
    }
    ++loop_counter;
    Serial.print(digitalRead(2));
    Serial.print(" ");
    Serial.println(loop_counter);
}
