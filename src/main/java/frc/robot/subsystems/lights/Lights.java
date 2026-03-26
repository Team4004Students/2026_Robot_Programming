// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Lights extends SubsystemBase {
  private static I2C backArduino = new I2C(Port.kMXP, LightConstants.BACK_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C shooterArduino = new I2C(Port.kMXP, LightConstants.SHOOTER_ARDUINO_DEVICE_ADDRESS.getValue());
  private static I2C intakeArduino = new I2C(Port.kMXP, LightConstants.INTAKE_ARDUINO_DEVICE_ADDRESS.getValue());

  public static void turnOffIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnBlueIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_LIGHT_COMMAND.getValue());}
  public static void turnRedIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_LIGHT_COMMAND.getValue());}
  public static void turnMarsIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.MARS_LIGHT_COMMAND.getValue());}
  public static void turnRedAniIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_ANI_LIGHT_COMMAND.getValue());}
  public static void turnBlueAniIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_ANI_LIGHT_COMMAND.getValue());}

   public static void turnOffShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnBlueShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_LIGHT_COMMAND.getValue());}
  public static void turnRedShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_LIGHT_COMMAND.getValue());}
  public static void turnMarsShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.MARS_LIGHT_COMMAND.getValue());}
  public static void turnRedAniShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_ANI_LIGHT_COMMAND.getValue());}
  public static void turnBlueAniShooter() {shooterArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_ANI_LIGHT_COMMAND.getValue());}

  public static void turnOffBack() {backArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnRedBack() {backArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.RED_LIGHT_COMMAND.getValue());}
  public static void turnMarsBack() {backArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.MARS_LIGHT_COMMAND.getValue());}
  public static void turnBlueForOrangeBack() {backArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BLUE_FOR_ORANGE_LIGHT_COMMAND.getValue());}

 /*  public static void turnOffElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.OFF_LIGHT_COMMAND.getValue());}
  public static void turnZeroElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.INTAKE_BLUE_LIGHT_COMMAND.getValue());}
  public static void turnOneElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.INTAKE_RED_LIGHT_COMMAND.getValue());}
  public static void turnTwoElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.INTAKE_RED_ANI_LIGHT_COMMAND.getValue());}
  public static void turnThreeElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.INTAKE_BLUE_ANI_LIGHT_COMMAND.getValue());}
  public static void turnFourElevator() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.INTAKE_OFF_LIGHT_COMMAND.getValue());}
  public static void turnClimberAttachment() {elevatorArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.SHOOTER_BLUE_LIGHT_COMMAND.getValue());}
  public static void turnVerticalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.SHOOTER_RED_LIGHT_COMMAND.getValue());}
  public static void turnVerticalEjectIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.SHOOTER_RED_ANI_LIGHT_COMMAND.getValue());}
  public static void turnIntakeHasGamePeice() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.SHOOTER_BLUE_ANI_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.SHOOTER_OFF_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BACK_BLUE_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BACK_RED_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BACK_ORANGE_LIGHT_COMMAND.getValue());}
  public static void turnHorizontalRunningIntake() {intakeArduino.write(LightConstants.ARDUINO_REGISTER_ADDRESS.getValue(), LightConstants.BACK_OFF_LIGHT_COMMAND.getValue());}*/

  public static void getAllianceLights(Shooter shooter, Intake intake) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get()==Alliance.Red){
        if (shooter.atSpeed()) {
          turnRedAniShooter();
          turnRedAniIntake();
        } else {
          turnRedShooter();
          turnRedIntake();
        }

        /*
        if (intake.intakeRunning) {
          turnRedAniIntake();
        } else {
          turnRedIntake();
        }
        */

        if (DriverStation.isAutonomous()) {
          turnMarsBack();
        } else {
          turnRedBack();
        }
      }

      if (ally.get()==Alliance.Blue) {
        if (shooter.atSpeed()) {
          turnBlueAniShooter();
          turnBlueAniIntake();
        } else {
          turnBlueShooter();
          turnBlueIntake();
        }

        /*
        if (intake.intakeRunning) {
          turnBlueAniIntake();
        } else {
          turnBlueIntake();
        }
        */
        
        if (DriverStation.isAutonomous()) {
          turnMarsBack();
        } else {
          turnBlueForOrangeBack();
        }
      }
    } else {
      turnOffIntake();
      turnOffShooter();
      turnOffBack();
    }
  }

  public Lights() {}

  @Override
  public void periodic() {
    
  }
}
