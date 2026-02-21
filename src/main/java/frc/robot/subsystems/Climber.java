// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder;

public class Climber extends SubsystemBase {
  private DigitalInput upperLimitSensor;
  private DigitalInput lowerLimitSensor;
  private SparkMax climberMotor;
  private SparkRelativeEncoder climberEncoder;
  private SparkClosedLoopController pidController;
  private double climberSpeed;
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new SparkMax(35, MotorType.kBrushed);
    climberEncoder = (SparkRelativeEncoder) climberMotor.getEncoder();
    pidController = climberMotor.getClosedLoopController();
    upperLimitSensor = new DigitalInput(2);
    lowerLimitSensor = new DigitalInput(3);

    SparkMaxConfig config = new SparkMaxConfig();

    config.closedLoop.p(0.1).i(0.0).d(0.0).maxMotion.cruiseVelocity(5000).maxAcceleration(3000).allowedProfileError(5);
    climberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void climberUp() {
    pidController.setSetpoint(10.0, ControlType.kMAXMotionPositionControl);
  }
  
  public void climberDown() {
    pidController.setSetpoint(0.0, ControlType.kMAXMotionPositionControl);
  } 
  
  public void climberStop() {
    pidController.setSetpoint(0.0, ControlType.kVoltage);
  }

  public boolean upperLimitSensor() {
    return upperLimitSensor.get();
  }

  public boolean lowerLimitSensor() {
    return lowerLimitSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
