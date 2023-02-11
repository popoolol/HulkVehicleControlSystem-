void setup_shift_registers(){
  // Declare arduino pins associated with shift registers 
  // reference list of pins
  const byte stab_shiftreg_data   = 12;  // stab Shift Register data
  const byte stab_shiftreg_latch  = 10;  // stab Shift Register latch
  const byte stab_shiftreg_clock  = 11;  // stab Shift Register clock 
  const byte wheel_shiftreg_data  = 7;   // move Shift Register data
  const byte wheel_shiftreg_latch = 6;   // move Shift Register latch
  const byte wheel_shiftreg_clock = 5;   // move Shift Register clock
  // set those pins as output
  pinMode(stab_shiftreg_clock, OUTPUT); // 74HC595 Pins
  pinMode(stab_shiftreg_latch, OUTPUT);
  pinMode(stab_shiftreg_data,  OUTPUT);
  pinMode(wheel_shiftreg_clock,OUTPUT); // 74HC595 Pins
  pinMode(wheel_shiftreg_latch,OUTPUT);
  pinMode(wheel_shiftreg_data, OUTPUT); 
}

// Function to set the interface to step-direction control
// Sets proper flags 
void enable_stepdir(){
  use_stepdir = true;
  use_uart = false;
}

// Shift register protocol
// The HULK features two shift registers, one for the movement system and one for the stability system 
// These are called wheel_shiftreg and stab_shiftreg 
// This script defines the register pins, and the data that is written to the registers

// This function will pulse one or multiple drivers (depending on data sent)
// TMC2209 drivers take a step ONLY on a rising change. The function sends a step pin high
void send_stepdir_data(){
  if (control_wheels)
  {
      // Write pin high
      const byte wheel_shiftreg_data  = 7;   // move Shift Register data
      const byte wheel_shiftreg_latch = 6;   // move Shift Register latch
      const byte wheel_shiftreg_clock = 5;   // move Shift Register clock
      digitalWrite(wheel_shiftreg_latch, LOW);                                 // Open latch 
      shiftOut(wheel_shiftreg_data,wheel_shiftreg_clock,MSBFIRST, shift_data);  // Send Data
      digitalWrite(wheel_shiftreg_latch, HIGH);                                // Close latch
  }

  if (control_stabilizers)
  {
      const byte stab_shiftreg_data  = 12;  // stab Shift Register 1 data
      const byte stab_shiftreg_latch = 10;   // stab Shift Register 1 latch
      const byte stab_shiftreg_clock = 11;   // stab Shift Register 1 clock      
      digitalWrite(stab_shiftreg_latch, LOW);                                 // Open latch 
      shiftOut(stab_shiftreg_data,stab_shiftreg_clock,MSBFIRST, shift_data);  // Send Data
      digitalWrite(stab_shiftreg_latch, HIGH);                                // Close latch
      //delayMicroseconds(microseconds_per_step);
  }
}
