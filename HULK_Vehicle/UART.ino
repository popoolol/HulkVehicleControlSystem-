// Function to set the interface to uart control
// Sets proper flags 
void enable_uart(){
  use_stepdir = false;
  use_uart = true;

  // Stabilizers only: when switching control the address pin must be set 
  // HIGH for uart, LOW for step-dir
  if (control_stabilizers){
    stab_driver0.pdn_disable(true);
    stab_driver1.pdn_disable(true);
    stab_driver2.pdn_disable(true);
    stab_driver0.mstep_reg_select(true);
    stab_driver1.mstep_reg_select(true);
    stab_driver2.mstep_reg_select(true);
    shift_data = stabilizer_en_pin;
    send_stepdir_data();
    shift_data = address_pin; 
    send_stepdir_data();
  }
}


// send uart is meant to send vactual speeds to wheels and stabilizers
// 
void send_uart_data(){
//  Serial.print(" NORM: ");Serial.print(joy_norm);
//  Serial.print(" maxual: ");Serial.print(max_vactual);
  
  if (control_wheels){
//    Serial.print(" SENT W: ");Serial.print(vactual_speed);
    wheel_driver0.VACTUAL(vactual_speed);
    wheel_driver1.VACTUAL(vactual_speed);
    wheel_driver2.VACTUAL(vactual_speed);
    wheel_driver3.VACTUAL(vactual_speed);
  }
  if (control_stabilizers){
//    Serial.print(" SENT S: ");Serial.print(vactual_speed);
    stab_driver0.VACTUAL(vactual_speed); 
    stab_driver1.VACTUAL(vactual_speed);
    stab_driver2.VACTUAL(vactual_speed);
  }
  
}
