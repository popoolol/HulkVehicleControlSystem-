// Function to set the interface to uart control
// Sets proper flags 
void enable_uart(){
  use_stepdir = false;
  use_uart    = true;
}


// send uart is meant to send vactual speeds to wheels and stabilizers

void send_uart_data(){
  if (control_wheels)
  {
    wheel_driver0.VACTUAL(vactual_speed);
    wheel_driver1.VACTUAL(vactual_speed);
    wheel_driver2.VACTUAL(vactual_speed);
    wheel_driver3.VACTUAL(vactual_speed);
  }
  if (control_stabilizers)
  {
    stab_driver0.VACTUAL(vactual_speed); 
    stab_driver1.VACTUAL(vactual_speed);
    stab_driver2.VACTUAL(vactual_speed);
  } 
}
