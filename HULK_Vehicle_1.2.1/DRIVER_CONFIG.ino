// TMC2209 SETUP 
// initializer serial connections, declare baud rates 
// Function run in setup()
void setup_stabilizers(){
   
    // Start by disabling drivers. It appears they have communication errors if the UART connection 
    // is established while the drivers are enabled
    byte enable_pin = 4;            // Arduino Pin for enabling   
    pinMode(enable_pin, OUTPUT);    // Set pinmode 
    digitalWrite(enable_pin,HIGH);  
    
    uint32_t uart_baud_rate = 115200;
    // Begin Software Serial communication
    stab_soft_serial.begin(115200);   // initialize software serial for UART motor control
    //delay(10);
    // Begin driver communication, defining each address
    stab_driver0.beginSerial(115200); // Initialize UART Motor 0
    //delay(10);
    stab_driver1.beginSerial(115200); // Initialize UART Motor 1
    //delay(10);
    stab_driver2.beginSerial(115200); // Initialize UART Motor 2
    //delay(10);

    // Stabilizers need to be individually tuned for sensing loads
    // Following values were determined through trial and error
    // Need more testing to fine tune further 
    uint16_t stabilizer0_current   = 800;
    uint16_t stabilizer1_current   = 800;
    uint16_t stabilizer2_current   = 1500;
    uint8_t  set_microsteps        = 0;
    uint8_t  stall0_threshold      = 125;
    uint8_t  stall1_threshold      = 175;
    uint8_t  stall2_threshold      = 80;
    
    // STABILITY MOTOR 0 //
    stab_driver0.begin();  
    //delay(10);
    stab_driver0.pdn_disable(true);                    
    stab_driver0.toff(5);                        
    stab_driver0.rms_current(stabilizer0_current);    
    stab_driver0.microsteps(set_microsteps);  
    stab_driver0.I_scale_analog(false);
    stab_driver0.vsense(false);
    stab_driver0.en_spreadCycle(false);
    stab_driver0.pwm_autoscale(true);   
    stab_driver0.SGTHRS(stall0_threshold);
    
    // STABILITY MOTOR 1 // ENABLE & SET CURRENT
    stab_driver1.begin();
    //delay(10);
    stab_driver1.toff(5);                
    stab_driver1.rms_current(stabilizer1_current);       
    stab_driver1.microsteps(set_microsteps);          
    stab_driver1.I_scale_analog(false);
    stab_driver1.vsense(false);
    stab_driver1.en_spreadCycle(false);
    stab_driver1.pwm_autoscale(true);    
    stab_driver1.SGTHRS(stall1_threshold);
    
    // STABILITY MOTOR 2 // ENABLE & SET CURRENT
    stab_driver2.begin();
    //delay(10);
    stab_driver2.toff(5);                
    stab_driver2.rms_current(stabilizer2_current);       
    stab_driver2.microsteps(set_microsteps);          
    stab_driver2.I_scale_analog(false);
    stab_driver2.en_spreadCycle(false);
    stab_driver2.pwm_autoscale(true);    // Needed for stealthChop
    stab_driver2.SGTHRS(stall2_threshold);

    digitalWrite(enable_pin,LOW);
}


// AMIS-30543 SETUP
void setup_wheels()
{
    const byte ChipSelect = 3;
    
    SPI.begin();
    stepper.init(ChipSelect);
    
    // Give the driver some time to power up.
    delay(1);
  
    // Reset the driver to its default settings.
    stepper.resetSettings();
  
    // Set the current limit.  You should change the number here to
    // an appropriate value for your particular system.
    stepper.setCurrentMilliamps(2200);
  
    // Set the number of microsteps that correspond to one full step.
    stepper.setStepMode(4);
  
    // Enable the motor outputs.
    stepper.enableDriver();
}
