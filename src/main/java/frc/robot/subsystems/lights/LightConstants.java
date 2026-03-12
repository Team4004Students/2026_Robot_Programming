// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

 
  /** Creates a new LightConstants. */
  public enum LightConstants {
    INTAKE_ARDUINO_DEVICE_ADDRESS(4),
    SHOOTER_ARDUINO_DEVICE_ADDRESS(5),
    BACK_ARDUINO_DEVICE_ADDRESS(6),
    ARDUINO_REGISTER_ADDRESS(0),
    OFF_LIGHT_COMMAND(0),
    BLUE_LIGHT_COMMAND(3),
    BLUE_FOR_ORANGE_LIGHT_COMMAND(3),
    RED_LIGHT_COMMAND(1),
    MARS_LIGHT_COMMAND(3),
    RED_ANI_LIGHT_COMMAND(2),
    BLUE_ANI_LIGHT_COMMAND(4);
   /* INTAKE_BLUE_LIGHT_COMMAND(3),
    INTAKE_RED_LIGHT_COMMAND(1),
    INTAKE_RED_ANI_LIGHT_COMMAND(2),
    INTAKE_BLUE_ANI_LIGHT_COMMAND(4),
    INTAKE_OFF_LIGHT_COMMAND(0),
    SHOOTER_BLUE_LIGHT_COMMAND(3),
    SHOOTER_RED_LIGHT_COMMAND(1),
    SHOOTER_RED_ANI_LIGHT_COMMAND(2),
    SHOOTER_BLUE_ANI_LIGHT_COMMAND(4),
    SHOOTER_OFF_LIGHT_COMMAND(0),
    BACK_BLUE_LIGHT_COMMAND(2),
    BACK_RED_LIGHT_COMMAND(1),
    BACK_ORANGE_LIGHT_COMMAND(3),
    BACK_OFF_LIGHT_COMMAND(0);*/
    

    private int value = 0;

    public int getValue() {
        return value;
    }

    private LightConstants(int assignedValue) {
        value = assignedValue;
   }

}
