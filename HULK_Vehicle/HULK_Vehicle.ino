#include <TMCStepper.h>     // for motor driver control
#include <SoftwareSerial.h> // For software serial
#include <Wire.h>           // for accelerometer control

// SERIAL CONNECITON RATES //
const int serial_baud_rate  = 9600;  // Baud rate for print statements(between arduino and pc)
const long uart_baud_rate   = 115200;// Baud rate for uart connection

// SOFTWARE SERIAL PINS, DEFINITIONS //
// Define UART Pins
const int stab_tx_pin = 9;  // UART transmission pin
const int stab_rx_pin = 3;   // UART receiving pin
const int wheel_tx_pin = 8;  // UART transmission pin

////////////////////////////////////////////////////////////////////////////////
// TEMPORARILY CHANGED TO SUPPORT TWO JOYSTICK BUTTONS!!!!!!!!!
const int wheel_rx_pin = 2;   // UART receiving pin
//const int wheel_rx_pin = 22;   // UART receiving pin
//const int joy1_b_pin = 2;   // UART receiving pin
//////////////////////////////////////////////////////////////////////////////

// Enabling Software Serial Connection
SoftwareSerial stab_soft_serial(stab_rx_pin, stab_tx_pin);  // Be sure to connect RX to TX with resistor. Driver has one pin
SoftwareSerial wheel_soft_serial(wheel_rx_pin, wheel_tx_pin);  // Be sure to connect RX to TX with resistor. Driver has one pin

// MOTOR DRIVER HARDWARE, VARIABLES, DECLARATIONS //

// TMC2209 Driver Hardware information
const int stab_driver_address0  = 0;    // stability Driver addresses depend on MS1, MS2 setup
const int stab_driver_address1  = 1;    // MS1 and MS2 set high with 8th shift register bit 
const int stab_driver_address2  = 2;
const int wheel_driver_address0 = 0;    // movement Driver addresses depend on MS1, MS2 setup
const int wheel_driver_address1 = 1;    // MS1 and MS2 set high using jumper pin on board 
const int wheel_driver_address2 = 2;
const int wheel_driver_address3 = 3;
const float r_sense             = 0.11; // external sense resistor value in ohms, used in current calculation
const int num_microsteps        = 0;    // Sets drivers to full stepping, 
// Variables 
int rms_current        = 200;  // run current mA
int detect_stall_value = 100; // This is what sg_result is compared to, a lower number needs more torque to indicate a stall [0..255]
// Instantiating motor drivers (0-3 labeled on circuit)
TMC2209Stepper stab_driver0(&stab_soft_serial, r_sense, stab_driver_address0);   // stability driver 0
TMC2209Stepper stab_driver1(&stab_soft_serial, r_sense, stab_driver_address1);   // stability driver 1
TMC2209Stepper stab_driver2(&stab_soft_serial, r_sense, stab_driver_address2);   // stability driver 2
TMC2209Stepper wheel_driver0(&wheel_soft_serial, r_sense, wheel_driver_address0);   // movement driver 0
TMC2209Stepper wheel_driver1(&wheel_soft_serial, r_sense, wheel_driver_address1);   // movement driver 1
TMC2209Stepper wheel_driver2(&wheel_soft_serial, r_sense, wheel_driver_address2);   // movement driver 2
TMC2209Stepper wheel_driver3(&wheel_soft_serial, r_sense, wheel_driver_address3);   // movement driver 3

// JOYSTICK PINS, VARIABLES, FLAGS // 
// Define joysitck pins
int joy0_x_pin = A0;  // Analog input pins(joysticks) 
int joy0_y_pin = A1;
int joy1_x_pin = A2;
int joy1_y_pin = A3;
int joy0_b_pin = 13;   // Button pin inputs 
 
//int joy1_b_pin = 25; //////////////////////////////////////CHANGE BACK WHEN DONE TESTING!!!!!

// Joystick variables 
float joy0_x_input, joy0_y_input, joy0_b_input;   // Joystick 0 variables (global because shared between functions)
float joy1_x_input, joy1_y_input, joy1_b_input;   // Joystick 1 variables 
float max_joy_value = 512;                        // maximum allowed joystick value, joysticks can be +- this value
float joy_norm;                                   // Step speeds are calculated based off a maximum allowed speed. The joystick values are normalized and compared to the speed value
int joy0_x_shift, joy0_y_shift;                   // Joysticks may not read in the center, this will determine an offset at starup
int joy1_x_shift, joy1_y_shift;
int deadzone = 30;                                // Deadzone is a region where the joystick inputs are ignored 
unsigned long last_single_button_time = 0;        // Button debouncer for single button press
unsigned long last_double_button_time = 0;        // Button debouncer for double button press
const int debounce_time = 300;                    // set desired debounce limit in ms
bool button0_state;                               // Current polled states of the joystick buttons 
bool button1_state;
// Joystick flags 
bool use_uart;             // swtich between uart or step-direction interfaces 
bool use_stepdir;         // step-direction interface is set as false to begin
bool manual_control;       // set flag for manual control 
bool run_autoleveling = false;   // set flag for running the auto_levelig function

// STABILIZER & WHEEL PINS VARIABLES, FLAGS // (INCLUDES ACCELEROMETER INFORMATION)
// Define Acceleromter information
const int accel_register = 0x68; // register designation for reading accelerometer
int16_t accel_x_input,accel_y_input;                 // first 16 bits of register is x data, next 16 is y data
int accel_x_shift, accel_y_shift;
const int end_leveling_value = 100;   // minimum accelerometer value for determininng a minimum
// Stabilizer Variables 
long stabilizer0_step_count; // counts the steps taken for each stabilizer
long stabilizer1_step_count;
long stabilizer2_step_count;
int raise_count = 200;       // number of steps taken once the feet tocuh the ground. This is one full rotation
int stall_value = 150;
int low_side;                // Determines which of the 4 sides is lowest 
// Stabilizer Flags 
bool stabilizer0_home = false;
bool stabilizer1_home = false;
bool stabilizer2_home = false;
bool all_stabilizers_home = false;
bool stabilizer0_down = false;
bool stabilizer1_down = false;
bool stabilizer2_down = false;
bool all_stabilizers_down = false;
bool is_leveling = false;
bool control_stabilizers; // switch between controlling movement and stability systems, starts controlling movement  

// Wheel Pins
const int wheel_en_pin = 4;  
/// Wheel Flags  
bool control_wheels;       // controlling movement is initialized true

// SHIFT REGISTER PINS, VARIABLES //
// Define Shift Register Pins (Step-direction)
const int stab_shiftreg_data  = 12;  // stab Shift Register 1 data
const int stab_shiftreg_latch = 10;   // stab Shift Register 1 latch
const int stab_shiftreg_clock = 11;   // stab Shift Register 1 clock
const int wheel_shiftreg_data  = 7;   // move Shift Register data
const int wheel_shiftreg_latch = 6;   // move Shift Register 1 latch
const int wheel_shiftreg_clock = 5;   // move Shift Register 1 clock
// Bit references for the shift register, identifying register pin outputs 
const int dir0    = 1;    //00000001
const int step0   = 2;    //00000010
const int dir1    = 4;    //00000100
const int step1   = 8;    //00001000
const int dir2    = 16;   //00010000
const int step2   = 32;   //00100000
const int dir3    = 64;   //01000000
const int step3   = 128;  //10000000
// Registers 0-6 are identical between systems. 7-8 are different. The movement systems needs 7-8 to control the fourth motor. The stability 
// system only has 3 motors, leaving those pins open. They are the enable pin and the address pin for the stability system.
// The enable pin is a dedicated arduino pin on the movement system, and the MS1 and MS2 addresses are set high on the movement system using a jumper pin on the cicuit. 
// Setting the 7-8 bit on the stabilty system will set the MS1 and MS2 pins high, designating driver addresses, as well as setting the enable pin
const int stabilizer_en_pin  = 64; //01000000
const int address_pin = 128;//10000000
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
bool shaft_direction; // true turns motors forward, false turns motors backward. 

unsigned long last_time = 0;
unsigned long loop_time;

//== Setup ===============================================================================
void setup() {
  // Setup Arduino Output pins
  pinMode(stab_shiftreg_clock,OUTPUT); // 74HC595 Pins
  pinMode(stab_shiftreg_latch,OUTPUT);
  pinMode(stab_shiftreg_data,OUTPUT);
  pinMode(wheel_shiftreg_clock,OUTPUT); // 74HC595 Pins
  pinMode(wheel_shiftreg_latch,OUTPUT);
  pinMode(wheel_shiftreg_data,OUTPUT);
  pinMode(joy0_b_pin, INPUT_PULLUP);  // joystick button pin
//  pinMode(joy1_b_pin, INPUT_PULLUP);  
  pinMode(wheel_en_pin, OUTPUT);      // Dedicated wheel driver enable pin 

  //enable_wheels();
  enable_stabilizers();
  enable_uart();

//  // Begin accelerometer communication
  Wire.begin();               
  Wire.beginTransmission(accel_register); // selecting register to read and write to
  Wire.write(0x6B); // hex protocall for reading and writing to acceleromter
  Wire.write(0);    // clear register
  Wire.endTransmission(true); // end communication
  
  // BEGIN SERIAL COMMUNICATIONS //
  Serial.begin(uart_baud_rate);           // initialize hardware serial for debugging
  wheel_soft_serial.begin(uart_baud_rate);   // initialize software serial for UART motor control
  stab_soft_serial.begin(uart_baud_rate);   // initialize software serial for UART motor control
  stab_driver0.beginSerial(uart_baud_rate); // Initialize UART Motor 0
  stab_driver1.beginSerial(uart_baud_rate); // Initialize UART Motor 1
  stab_driver2.beginSerial(uart_baud_rate); // Initialize UART Motor 2
  wheel_driver0.beginSerial(uart_baud_rate); // Initialize UART Motor 0
  wheel_driver1.beginSerial(uart_baud_rate); // Initialize UART Motor 1
  wheel_driver2.beginSerial(uart_baud_rate); // Initialize UART Motor 2
  wheel_driver3.beginSerial(uart_baud_rate); // Initialize UART Motor 3
 
  // Setup driver values
  // STABILITY MOTOR 0 //
  stab_driver0.begin(); // Begin connection
  stab_driver0.toff(5); // Enables driver in software
  stab_driver0.rms_current(rms_current);    // Set motor RMS current
  stab_driver0.microsteps(num_microsteps);  // Set microsteps
  stab_driver0.en_spreadCycle(false);
  stab_driver0.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  stab_driver0.SGTHRS(stall_value);
  // STABILITY MOTOR 1 // ENABLE & SET CURRENT
  stab_driver1.begin();
  stab_driver1.toff(5);                // Enables driver in software
  stab_driver1.rms_current(rms_current);       // Set motor RMS current
  stab_driver1.microsteps(num_microsteps);          // Set microsteps
  stab_driver1.en_spreadCycle(false);
  stab_driver1.pwm_autoscale(true);    // Needed for stealthChop
  stab_driver1.SGTHRS(stall_value);
  // STABILITY MOTOR 2 // ENABLE & SET CURRENT
  stab_driver2.begin();
  stab_driver2.toff(5);                // Enables driver in software
  stab_driver2.rms_current(rms_current);       // Set motor RMS current
  stab_driver2.microsteps(num_microsteps);          // Set microsteps
  stab_driver2.en_spreadCycle(false);
  stab_driver2.pwm_autoscale(true);    // Needed for stealthChop
  stab_driver2.SGTHRS(stall_value);
  // MOVEMENT MOTOR 0 //
  wheel_driver0.begin();
  wheel_driver0.toff(5);               // Enables driver in software
  wheel_driver0.rms_current(rms_current);      // Set motor RMS current
  wheel_driver0.microsteps(num_microsteps);         // Set microsteps
  wheel_driver0.en_spreadCycle(false);
  wheel_driver0.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  // MOVEMENT MOTOR 1 //
  wheel_driver1.begin(); 
  wheel_driver1.toff(5);               // Enables driver in software
  wheel_driver1.rms_current(rms_current);      // Set motor RMS current
  wheel_driver1.microsteps(num_microsteps);         // Set microsteps
  wheel_driver1.en_spreadCycle(false);
  wheel_driver1.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  // MOVEMENT MOTOR 2 //
  wheel_driver2.begin();
  wheel_driver2.toff(5);               // Enables driver in software
  wheel_driver2.rms_current(rms_current);      // Set motor RMS current
  wheel_driver2.microsteps(num_microsteps);         // Set microsteps
  wheel_driver2.en_spreadCycle(false);
  wheel_driver2.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  // MOVEMENT MOTOR 3 //
  wheel_driver3.begin();
  wheel_driver3.toff(5);               // Enables driver in software
  wheel_driver3.rms_current(rms_current);      // Set motor RMS current
  wheel_driver3.microsteps(num_microsteps);         // Set microsteps
  wheel_driver3.en_spreadCycle(false);
  wheel_driver3.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change


  // Setup joysticks by checking initial offsets
  joy0_x_shift = map(analogRead(joy0_x_pin), 0, 1024, -max_joy_value, max_joy_value); // Gets an initial offset for the joystick poteniometer to correct values
  joy0_y_shift = map(analogRead(joy0_y_pin), 0, 1024, -max_joy_value, max_joy_value);
  joy1_x_shift = map(analogRead(joy1_x_pin), 0, 1024, -max_joy_value, max_joy_value);
  joy1_y_shift = map(analogRead(joy1_y_pin), 0, 1024, -max_joy_value, max_joy_value);

  Wire.beginTransmission(accel_register);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(accel_register,6,true);  //Reading 6 bytes from MPU
  accel_x_shift = Wire.read()<<8|Wire.read();    
  accel_y_shift = Wire.read()<<8|Wire.read();

  Serial.print("END SETUP");
}

//== MAIN LOOP ===============================================================================
void loop() {
  
  if (manual_control) {read_joysticks();} // Only read joystick input if manual mode is selected 
  //Serial.println();

}
