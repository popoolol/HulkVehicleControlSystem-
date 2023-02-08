// Function to set the interface to step-direction control
// Sets proper flags 
void enable_stepdir(){
  use_stepdir = true;
  use_uart = false;

  
  // Stabilizers only: when switching between control interfaces, the address pin must be set 
  // LOW for step-dir, HIGH for uart
  if (control_stabilizers){
    stab_driver0.pdn_disable(false);
    stab_driver1.pdn_disable(false);
    stab_driver2.pdn_disable(false);
    stab_driver0.mstep_reg_select(false);
    stab_driver1.mstep_reg_select(false);
    stab_driver2.mstep_reg_select(false);
    shift_data = stabilizer_en_pin;
    send_stepdir_data();
    shift_data = 0;
    send_stepdir_data();
  }
}

// Shift register protocol
// The HULK features two shift registers, one for the movement system and one for the stability system 
// These are called wheel_shiftreg and stab_shiftreg 
// This script defines the register pins, and the data that is written to the registers

// This function will pulse one or multiple drivers (depending on data sent)
// TMC2209 drivers take a step ONLY on a rising change. The function sends a step pin high
void send_stepdir_data(){
//  Serial.print(" NORM: ");Serial.print(joy_norm);
//  Serial.print(" minstep: ");Serial.print(min_microseconds_per_step);

  
  if (control_wheels){
//      Serial.print(" SENT W: ");Serial.print(microseconds_per_step);
      // Write pin high
      digitalWrite(wheel_shiftreg_latch, LOW);                                 // Open latch 
      shiftOut(wheel_shiftreg_data,wheel_shiftreg_clock,MSBFIRST, shift_data);  // Send Data
      digitalWrite(wheel_shiftreg_latch, HIGH);                                // Close latch
//      delayMicroseconds(microseconds_per_step);
//      Serial.print(" BITS MSF: ");
//      Serial.print(bitRead(shift_data, 11));Serial.print(bitRead(shift_data, 10));Serial.print(bitRead(shift_data, 9));Serial.print(bitRead(shift_data, 8));
//      Serial.print("|");
//      Serial.print(bitRead(shift_data, 7));Serial.print(bitRead(shift_data, 6));Serial.print(bitRead(shift_data, 5));Serial.print(bitRead(shift_data, 4));
//      Serial.print(bitRead(shift_data, 3));Serial.print(bitRead(shift_data, 2));Serial.print(bitRead(shift_data, 1));Serial.print(bitRead(shift_data, 0));
  }

  if (control_stabilizers){
//      Serial.print(" SENT S: ");Serial.print(microseconds_per_step);
      // Write pin high
      
      digitalWrite(stab_shiftreg_latch, LOW);                                 // Open latch 
      shiftOut(stab_shiftreg_data,stab_shiftreg_clock,MSBFIRST, shift_data);  // Send Data
      digitalWrite(stab_shiftreg_latch, HIGH);                                // Close latch
      //delayMicroseconds(microseconds_per_step);
  }
  
  
}
