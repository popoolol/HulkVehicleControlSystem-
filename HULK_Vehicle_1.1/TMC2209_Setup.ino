 void begin_driver_serial(){
   
  const long uart_baud_rate = 115200;
  // Begin Software Serial communication
  wheel_soft_serial.begin(uart_baud_rate);   // initialize software serial for UART motor control
  stab_soft_serial.begin (uart_baud_rate);   // initialize software serial for UART motor control
  // Begin driver communication, defining each address
  wheel_driver0.beginSerial(uart_baud_rate); // Initialize UART Motor 0
  wheel_driver1.beginSerial(uart_baud_rate); // Initialize UART Motor 1
  wheel_driver2.beginSerial(uart_baud_rate); // Initialize UART Motor 2
  wheel_driver3.beginSerial(uart_baud_rate); // Initialize UART Motor 2
  stab_driver0.beginSerial (uart_baud_rate); // Initialize UART Motor 0
  stab_driver1.beginSerial (uart_baud_rate); // Initialize UART Motor 1
  stab_driver2.beginSerial (uart_baud_rate); // Initialize UART Motor 2
}

void wheel_driver_setup(){  
  
  int wheel_current = 1770;
  int wheel_microsteps = 0;
 
  // MOVEMENT MOTOR 0 //
  wheel_driver0.begin();
  wheel_driver0.toff(5);                      // Enables driver in software
  wheel_driver0.rms_current(wheel_current);   // Set motor RMS current
  wheel_driver0.I_scale_analog(false);
  wheel_driver0.vsense(false);
  wheel_driver0.pdn_disable(false);
  wheel_driver0.microsteps(wheel_microsteps);   // Set microsteps
  wheel_driver0.en_spreadCycle(false);
  wheel_driver0.pwm_autoscale(false);         // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  wheel_driver0.pwm_ofs(255);
  // MOVEMENT MOTOR 1 //
  wheel_driver1.begin(); 
  wheel_driver1.toff(5);               
  wheel_driver1.rms_current(wheel_current);      
  wheel_driver1.microsteps(wheel_microsteps);         
  wheel_driver1.en_spreadCycle(false);
  wheel_driver1.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  // MOVEMENT MOTOR 2 //
  wheel_driver2.begin();
  wheel_driver2.toff(5);               // Enables driver in software
  wheel_driver2.rms_current(wheel_current);      // Set motor RMS current
  wheel_driver2.microsteps(wheel_microsteps);         // Set microsteps
  wheel_driver2.en_spreadCycle(false);
  wheel_driver2.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  // MOVEMENT MOTOR 3 //
  wheel_driver3.begin();
  wheel_driver3.toff(5);               // Enables driver in software
  wheel_driver3.rms_current(wheel_current);      // Set motor RMS current
  wheel_driver3.microsteps(wheel_microsteps);         // Set microsteps
  wheel_driver3.en_spreadCycle(false);
  wheel_driver3.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
}

void stabilizer_driver_setup(){

  // Driver variables 
  int stabilizer0_current    = 800;
  int stabilizer1_current    = 800;
  int stabilizer2_current    = 1500;
  int stabilizer_microsteps  = 0;
  int stall0_threshold       = 125;
  int stall1_threshold       = 175;
  int stall2_threshold       = 100;

  
  // Setup driver values
  // STABILITY MOTOR 0 //
  stab_driver0.begin(); // Begin connection
  stab_driver0.toff(5); // Enables driver in software
  stab_driver0.rms_current(stabilizer0_current);    // Set motor RMS current
  stab_driver0.microsteps(stabilizer_microsteps);  // Set microsteps
  stab_driver0.I_scale_analog(false);
  stab_driver0.vsense(false);
  stab_driver0.pdn_disable(false);
  stab_driver0.en_spreadCycle(false);
  stab_driver0.pwm_autoscale(true);   // Needed for stealthChop (StealthChop is always enabled by hardware need to solder jumper to change
  stab_driver0.SGTHRS(stall0_threshold);
  // STABILITY MOTOR 1 // ENABLE & SET CURRENT
  stab_driver1.begin();
  stab_driver1.toff(5);                // Enables driver in software
  stab_driver1.rms_current(stabilizer1_current);       // Set motor RMS current
  stab_driver1.microsteps(stabilizer_microsteps);          // Set microsteps
  stab_driver1.I_scale_analog(false);
  stab_driver1.vsense(false);
  stab_driver1.pdn_disable(false);
  stab_driver1.en_spreadCycle(false);
  stab_driver1.pwm_autoscale(true);    // Needed for stealthChop
  stab_driver1.SGTHRS(stall1_threshold);
  // STABILITY MOTOR 2 // ENABLE & SET CURRENT
  stab_driver2.begin();
  stab_driver2.toff(5);                // Enables driver in software
  stab_driver2.rms_current(stabilizer2_current);       // Set motor RMS current
  stab_driver2.microsteps(stabilizer_microsteps);          // Set microsteps
  stab_driver2.I_scale_analog(false);
  stab_driver2.vsense(false);
  stab_driver2.pdn_disable(false);
  stab_driver2.en_spreadCycle(false);
  stab_driver2.pwm_autoscale(true);    // Needed for stealthChop
  stab_driver2.SGTHRS(stall2_threshold);
  stab_driver2.ihold(31);
}
