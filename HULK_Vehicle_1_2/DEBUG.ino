
void DEBUG_PRINT() {
//    Serial.print(" ax: ");Serial.print(accel_x_input);
//    Serial.print(" ay: ");Serial.print(accel_y_input);
//      Serial.print(" x0: ");Serial.print(joy0_x_input);
//      Serial.print(" y0: ");Serial.print(joy0_y_input);
//      Serial.print(" x1: ");Serial.print(joy1_x_input);
//      Serial.print(" y1: ");Serial.print(joy1_y_input);
  //    Serial.print(" LS: ");Serial.print(digitalRead(5));
  //    Serial.print(" b0: ");Serial.print(button0_state);
  //    Serial.print(" b1: ");Serial.print(button1_state);
//    Serial.print(" AT: ");Serial.print(bitRead(system_bool, 7));
//    Serial.print(" DE: ");Serial.print(bitRead(system_bool, 6));
//    Serial.print(" MC: ");Serial.print(bitRead(system_bool, 5));
//    Serial.print(" AU: ");Serial.print(bitRead(system_bool, 4));
    Serial.print(" WH: ");Serial.print(bitRead(system_bool, 3));
    Serial.print(" ST: ");Serial.print(bitRead(system_bool, 2));
//    Serial.print(" PR: ");Serial.print(bitRead(system_bool, 1));
//    Serial.print(" MO: ");Serial.print(bitRead(system_bool, 0));
//    Serial.print(" LS: ");Serial.print(digitalRead(5));
  //    Serial.print(" b0: ");Serial.print(button0_state);
  //    Serial.print(" b1: ");Serial.print(button1_state);
//    Serial.print(" LE: ");Serial.print(bitRead(stabilizer_bool, 7));
//    Serial.print(" HO: ");Serial.print(bitRead(stabilizer_bool, 6));
//    Serial.print(" LD: ");Serial.print(bitRead(stabilizer_bool, 5));
//    Serial.print(" HD: ");Serial.print(bitRead(stabilizer_bool, 4));
//    Serial.print(" GR: ");Serial.print(bitRead(stabilizer_bool, 3));
//    Serial.print(" ST: "); Serial.print(bitRead(stabilizer_bool, 2));
//    Serial.print(" GD: ");Serial.print(bitRead(stabilizer_bool, 1));
//    Serial.print(" RA: "); Serial.print(bitRead(stabilizer_bool, 0));
//  Serial.print(" NC: "); Serial.print(now_controlling);
//  Serial.print(" PR: "); Serial.print(printing);


  //    Serial.print(" TEST: ");Serial.print(stab_driver0.test_connection());Serial.print(stab_driver1.test_connection());Serial.print(stab_driver2.test_connection());
  //    Serial.print(" DRV: ");Serial.print(stab_driver0.DRV_STATUS());Serial.print(stab_driver1.DRV_STATUS());Serial.print(stab_driver2.DRV_STATUS());
  //    Serial.print(" MSR: ");Serial.print(stab_driver0.mstep_reg_select());Serial.print(stab_driver1.mstep_reg_select());Serial.print(stab_driver2.mstep_reg_select());
  //    Serial.print(" PDN? ");Serial.print(stab_driver0.pdn_disable());Serial.print(stab_driver1.pdn_disable());Serial.print(stab_driver2.pdn_disable());
  //    Serial.print(" EN? ");Serial.print(stab_driver0.isEnabled());Serial.print(stab_driver1.isEnabled());Serial.print(stab_driver2.isEnabled());
  //    Serial.print(" V0: "); Serial.print(stab_driver0.VACTUAL());
  //    Serial.print(" V1: "); Serial.print(stab_driver1.VACTUAL());
  //    Serial.print(" V2: "); Serial.print(stab_driver1.VACTUAL());
  Serial.println();
}
