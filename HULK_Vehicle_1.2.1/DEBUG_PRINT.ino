
void DEBUG_PRINT() {
  // ANALOG DEVICES
  //    Serial.print(" ax: ");Serial.print(accel_x_input);
  //    Serial.print(" ay: ");Serial.print(accel_y_input);
//      Serial.print(" x0: ");Serial.print(joy0_x_input);
//      Serial.print(" y0: ");Serial.print(joy0_y_input);
//      Serial.print(" x1: ");Serial.print(joy1_x_input);
//      Serial.print(" y1: ");Serial.print(joy1_y_input);

  // Stabilizer stuff
  //    Serial.print(" LS: ");Serial.print(digitalRead(5));   // Limit switch state
  //    Serial.print(" NC: "); Serial.print(now_controlling); // Stabilizer currently being controlled
    

  // SYSTEM BOOL
//      Serial.print(" System:");
//      Serial.print(" PR: ");Serial.print(bitRead(system_bool, 7)); // Printing
//      Serial.print(" MO: ");Serial.print(bitRead(system_bool, 6)); // Moving
//      Serial.print(" AT: ");Serial.print(bitRead(system_bool, 5)); // Attached
//      Serial.print(" DE: ");Serial.print(bitRead(system_bool, 4)); // Detached
//      Serial.print(" MC: ");Serial.print(bitRead(system_bool, 3)); // Manual Control
//      Serial.print(" AU: ");Serial.print(bitRead(system_bool, 2)); // Automatic Control
//      Serial.print(" WH: ");Serial.print(bitRead(system_bool, 1)); // Wheel Control
//      Serial.print(" ST: ");Serial.print(bitRead(system_bool, 0)); // Stabilzier Control

  // STABILIZER BOOL
//    Serial.print(" Automatic:");
//      Serial.print(" LE: ");Serial.print(bitRead(auto_bool, 7)); // Leveling
//      Serial.print(" LD: ");Serial.print(bitRead(auto_bool, 6)); // Leveled
//      Serial.print(" HO: ");Serial.print(bitRead(auto_bool, 5)); // Homing
//      Serial.print(" HD: ");Serial.print(bitRead(auto_bool, 4)); // Homed
//      Serial.print(" GR: ");Serial.print(bitRead(auto_bool, 3)); // Grounding
//      Serial.print(" GD: ");Serial.print(bitRead(auto_bool, 2)); // Grounded
//    Serial.print(" PA: "); Serial.print(bitRead(auto_bool, 1)); // Pathing
//    Serial.print(" DN: "); Serial.print(bitRead(auto_bool, 0)); // Destination

  // Pathing
//  Serial.print(" PC: "); Serial.print(path_counter); // Current path


  // TMC2209 received infromation
  //    Serial.print(" TEST: ");Serial.print(stab_driver0.test_connection());Serial.print(stab_driver1.test_connection());Serial.print(stab_driver2.test_connection());
  //    Serial.print(" DRV: ");Serial.print(stab_driver0.DRV_STATUS());Serial.print(stab_driver1.DRV_STATUS());Serial.print(stab_driver2.DRV_STATUS());
  //    Serial.print(" MSR: ");Serial.print(stab_driver0.mstep_reg_select());Serial.print(stab_driver1.mstep_reg_select());Serial.print(stab_driver2.mstep_reg_select());
  //    Serial.print(" PDN? ");Serial.print(stab_driver0.pdn_disable());Serial.print(stab_driver1.pdn_disable());Serial.print(stab_driver2.pdn_disable());
  //    Serial.print(" EN? ");Serial.print(stab_driver0.isEnabled());Serial.print(stab_driver1.isEnabled());Serial.print(stab_driver2.isEnabled());
  //    Serial.print(" V0: "); Serial.print(stab_driver0.VACTUAL());
  //    Serial.print(" V1: "); Serial.print(stab_driver1.VACTUAL());
  //    Serial.print(" V2: "); Serial.print(stab_driver1.VACTUAL());


  Serial.println(); // New line
}
