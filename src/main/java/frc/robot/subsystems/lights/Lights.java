// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private static I2C driveTrainArduino = new I2C(Port.kMXP, LightConstants.DRIVETRAIN_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C elevatorArduino = new I2C(Port.kMXP, LightConstants.ELEVATOR_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C intakeArduino = new I2C(Port.kMXP, LightConstants.INTAKE_ARDUINO_DEVICE_ADDRESS.getValue());

  public static void turnOffDriveTrain() {driveTrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnBlueDriveTrain() {driveTrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_LIGHT_COMMAND.getValue());}
  public static void turnRedDriveTrain() {driveTrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_LIGHT_COMMAND.getValue());}
  public static void turnMarsDriveTrain() {driveTrainArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.MARS_LIGHT_COMMAND.getValue());}

  public static void turnOffElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnZeroElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.LEVEL_ZERO_ELEVATOR_LIGHT_COMMAND.getValue());}
  public static void turnOneElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.LEVEL_ONE_ELEVATOR_LIGHT_COMMAND.getValue());}
  public static void turnTwoElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.LEVEL_TWO_ELEVATOR_LIGHT_COMMAND.getValue());}
  public static void turnThreeElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.LEVEL_THREE_ELEVATOR_LIGHT_COMMAND.getValue());}
  public static void turnFourElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.LEVEL_FOUR_ELEVATOR_LIGHT_COMMAND.getValue());}
  public static void turnClimberAttachment() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.ATTACHMENT_ELEVATOR_LIGHT_COMMAND.getValue());}
  
  public static void turnOffIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnVerticalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.VERTICAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue());}
  public static void turnVerticalEjectIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.VERTICAL_RUNNING_EJECT_LIGHT_COMMAND.getValue());}
  public static void turnIntakeHasGamePeice() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.GAME_PIECE_INTAKE_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.HORIZONTAL_RUNNING_INTAKE_LIGHT_COMMAND.getValue());}
  
  public static void getAllianceLights() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()){
      if (ally.get()==Alliance.Red){
        turnRedDriveTrain();
      }
      if (ally.get()==Alliance.Blue){
        turnBlueDriveTrain();
      }
    } else {
      turnOffDriveTrain();
    }
  }



  public Lights() {}

  @Override
  public void periodic() {
    
  }
}
