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
    stepper.setCurrentMilliamps(2000);
  
    // Set the number of microsteps that correspond to one full step.
    stepper.setStepMode(4);
  
    // Enable the motor outputs.
    stepper.enableDriver();
}
