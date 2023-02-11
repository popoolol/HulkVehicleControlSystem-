#include <TMCStepper.h>     // for motor driver control
#include <SoftwareSerial.h> // For software serial
#include <Wire.h>           // for accelerometer control

// SOFTWARE SERIAL PINS, DEFINITIONS //
// Define UART Pins
const byte stab_tx_pin  = 9;  // UART transmission pin
const byte stab_rx_pin  = 3;  // UART receiving pin
const byte wheel_tx_pin = 8;  // UART transmission pin
const byte wheel_rx_pin = 2;  // UART receiving pin
// Enabling Software Serial Connection
SoftwareSerial stab_soft_serial(stab_rx_pin, stab_tx_pin);  // Be sure to connect RX to TX with resistor. Driver has one pin
SoftwareSerial wheel_soft_serial(wheel_rx_pin, wheel_tx_pin);  // Be sure to connect RX to TX with resistor. Driver has one pin

// TMC2209 Stabilizer Driver Hardware information
                         //(SERIAL PORT, SENSE RESISTOR VALUE, DRIVER ADDRESS)
TMC2209Stepper stab_driver0(&stab_soft_serial, 0.11f, 0);   // stability driver 0
TMC2209Stepper stab_driver1(&stab_soft_serial, 0.11f, 1);   // stability driver 1
TMC2209Stepper stab_driver2(&stab_soft_serial, 0.11f, 2);   // stability driver 2

// instantiate drivers 
TMC2209Stepper wheel_driver0(&wheel_soft_serial, 0.11f, 0);   // movement driver 0
TMC2209Stepper wheel_driver1(&wheel_soft_serial, 0.11f, 1);   // movement driver 1
TMC2209Stepper wheel_driver2(&wheel_soft_serial, 0.11f, 2);   // movement driver 2
TMC2209Stepper wheel_driver3(&wheel_soft_serial, 0.11f, 3);   // movement driver 3


// MOTOR DRIVER HARDWARE, VARIABLES, DECLARATIONS //
int sg_return = 0;


// JOYSTICK PINS, VARIABLES, FLAGS // 
// Joystick variables 
int joy0_x_input, joy0_y_input;   // Joystick 0 variables (global because shared between functions)
int joy1_x_input, joy1_y_input;   // Joystick 1 variables 
int8_t joy0_x_shift, joy0_y_shift;  // Joysticks may not read in the center, this will determine an offset at starup
int8_t joy1_x_shift, joy1_y_shift;
float joy_norm; // Step speeds are calculated based off a maximum allowed speed. The joystick values are normalized and compared to the speed value

bool button0_state;                        // Current polled states of the joystick buttons 
bool button1_state;
unsigned long last_single_button_time = 0; // Button debouncer for single button press

// Joystick flags 
bool use_uart;             // swtich between uart or step-direction interfaces 
bool use_stepdir;         // step-direction interface is set as false to begin
bool manual_control;       // set flag for manual control 
bool run_autoleveling = false;   // set flag for running the auto_levelig function

const byte sg_average_count = 3;
byte sg_running_average[sg_average_count];
const int accel_average_count = 5;
int accel_x_running_average[accel_average_count];
int accel_y_running_average[accel_average_count];
bool accel_is_offset = false;

// STABILIZER & WHEEL PINS VARIABLES, FLAGS // (INCLUDES ACCELEROMETER INFORMATION)
// Define Acceleromter information

int16_t accel_x_input, accel_y_input; // first 16 bits of register is x data, next 16 is y data
int     accel_x_shift, accel_y_shift;
const int end_leveling_value = 150;   // minimum accelerometer value for determininng a minimum
// Stabilizer Variables 
unsigned long stabilizer0_move_time; // counts the steps taken for each stabilizer
unsigned long stabilizer1_move_time;
unsigned long stabilizer2_move_time;
unsigned long stabilizer0_step_count; // counts the steps taken for each stabilizer
unsigned long stabilizer1_step_count;
unsigned long stabilizer2_step_count;
int raise_count = 200;       // number of steps taken once the feet tocuh the ground. This is one full rotation
int sg_smooth;
unsigned long loop_counter = 0;

int low_side;                // Determines which of the 4 sides is lowest 
// Stabilizer Flags 
bool stabilizer0_home = false;
bool stabilizer1_home = false;
bool stabilizer2_home = false;
bool all_stabilizers_home = false;
bool all_stabilizers_down = false;
bool is_leveling = false;
bool control_stabilizers; // switch between controlling movement and stability systems, starts controlling movement  

int now_controlling = 0;// used in the switch between function. This determines which motor/ stabilizer you are controlling
bool control_all;
bool control_indiv;

/// Wheel Flags  
bool control_wheels;       // controlling movement is initialized true

int shift_data;   // Data sent to shift register 

// STEP-DIRECTION DELAY CALCULATION //
// Step calculatios done using delay between step pulses, set desired RPM 
float max_stabilizer_rpm = 60; // <======== CHANGE THIS FOR MAX SPEED CAP
// Variables used to calculate the delay between pulses 
float steps_per_rev = 1600;       
float min_microseconds_per_step = round(60000000/max_stabilizer_rpm/steps_per_rev);  // calculates the new delay in microseconds
float max_microseconds_per_step = 16000;
unsigned int microseconds_per_step; // value given to the delaymicroseconds function to determine speed of rotation

// VACTUAL SPEED CALCULATION //
// vactual_speed is based on clock frequency and some maximum revolutions per second
float wheel_rps_max = 1.0; //<======== CHANGE THIS FOR MAXIMUM WHEEL REVOLUTIONS PER SECOND
// Use normalized joystick input to calculate vactual_speed. The normalized input is used to scale a desired maximum input  
float driver_clock_frequency = 12000000.0; // internal driver frequency 
float steps_per_revolution  = 200.0;      // Full steps per revolution 
float microstep_resolution  = 256.0;      // microstep resolution of the driver
float max_vactual = driver_clock_frequency * wheel_rps_max * steps_per_revolution * microstep_resolution / pow(2,24);
float vactual_speed;  // value that is given to the vactual function


unsigned long last_time = 0;
unsigned long loop_time;

//== Setup ===============================================================================
void setup() {
  // SERIAL CONNECITON RATES //

  const long uart_baud_rate   = 115200;// Baud rate for uart connection
  // BEGIN SERIAL COMMUNICATIONS //
  Serial.begin(112500);  // initialize hardware serial for debugging
  begin_driver_serial(); // initialize software serial and driver serial
  
  Serial.println();
  Serial.println();
  Serial.print("Established Serial Connections");
  Serial.println();
  
  // Setup Arduino Output pins
  setup_shift_registers();
  // enable pin for controlling wheels 
  const byte wheel_en_pin = 4;
  pinMode(wheel_en_pin, OUTPUT);  // Dedicated wheel driver enable pin
  Serial.print("Enabled Pins");
  Serial.println(); 

  // Choose starting system to control 
  //enable_wheels();
  enable_stabilizers();
  enable_uart();
  control_indiv = true;
  //control_all = true;
  Serial.print("Enabled System Control");
  Serial.println();

  // Begin accelerometer communication
  Wire.begin();                 // Begin accelerometer i2c connection
  Wire.beginTransmission(0x68); // This register resets the path of accelerometer data, as well as initializing interface
  Wire.write(0x6B);             // Writing to clock configuration register
  Wire.write(0);                // Byte 0 selects internal 8MHz clock
  Wire.endTransmission(true);   // end communication
  Serial.print("Completed Accel Communication");
  Serial.println();
  
  wheel_driver_setup();
  stabilizer_driver_setup();
  Serial.print("Setup Driver Values");
  Serial.println();

  get_joystick_offset();
  get_accel_offset();
  Serial.print("Retrieved Analog Device Offsets");
  Serial.println();

  Serial.print("END SETUP");
  Serial.println();
  Serial.println();
  delay(500);
}

//== MAIN LOOP ===============================================================================
void loop() {
  
  if (manual_control) {
    read_joysticks();       // Only read joystick input if manual mode is selected 
//    read_accel();
//    accel_smoother();
    if (!button0_state) {switch_between();}
    //system_and_interface();
    
      select_movement();
//      sg_smoother();
    
    
  } 
  various_print_statements();
  if (joy0_x_input < -50)
  { 
    autoleveling();
  }

 
  loop_counter++;
}

void various_print_statements(){
  Serial.print(" ax: ");Serial.print(accel_x_input);
  Serial.print(" ay: ");Serial.print(accel_y_input);
  Serial.print(" x0: ");Serial.print(joy0_x_input);
//  Serial.print(" y0: ");Serial.print(joy0_y_input);
//  Serial.print(" x1: ");Serial.print(joy1_x_input);
//  Serial.print(" y1: ");Serial.print(joy1_y_input);
//  Serial.print(" b0: ");Serial.print(button0_state);
//  Serial.print(" b1: ");Serial.print(button1_state);
//  Serial.print(" MC: ");Serial.print(manual_control);
//  Serial.print(" S: ");Serial.print(control_stabilizers);
//  Serial.print(" W: ");Serial.print(control_wheels);
//  Serial.print(" UA: ");Serial.print(use_uart);
//  Serial.print(" SD: ");Serial.print(use_stepdir);
//  Serial.print(" A: ");Serial.print(run_autoleveling);
  Serial.print(" C: ");Serial.print(now_controlling);
//  Serial.print(" SG : 100 THRS 150");
  
//  Serial.print(" VA: ");Serial.print(vactual_speed);
//    Serial.print(" SG0: ");
//  Serial.print(stab_driver0.SG_RESULT());
//  Serial.print(stab_driver0.cs_actual());
//  Serial.print(stab_driver0.SGTHRS());
//  Serial.print(" SG1: ");
//  Serial.print(stab_driver1.cs_actual());
//  Serial.print(stab_driver1.SG_RESULT());
//  Serial.print(stab_driver1.SGTHRS());
//  Serial.print(" SG2: ");
//  Serial.print(stab_driver2.cs_actual());
//  Serial.print(stab_driver2.SG_RESULT());
//  Serial.print(stab_driver2.SGTHRS());
//  Serial.print(" I RUN0: ");
//  Serial.print(stab_driver0.irun());

  Serial.println();
}
