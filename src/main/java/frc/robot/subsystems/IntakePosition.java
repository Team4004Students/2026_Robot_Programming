// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;



public class IntakePosition extends SubsystemBase {
  private TalonFXConfiguration config;
  private TalonFX intakePositionMotor;
  private DigitalInput bottomSensor;
  private DigitalInput topSensor;
  private CANcoder encoder;

  private NetworkTable intakePosTable = NetworkTableInstance.getDefault().getTable("intake position");

  /** Creates a new IntakePosition. */
  public IntakePosition() {
    intakePositionMotor = new TalonFX(32);
    config = new TalonFXConfiguration();
    //encoder = new CANcoder(24);

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //config.Feedback.FeedbackRemoteSensorID = 24;

     /** MotionMagic Configs */
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // Reach 5 (mechanism) rotations per second cruise.
               .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max velocity.
               .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max acceleration.

    /** PID Configs */
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction.
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output.
    slot0.kA = 0.01; // An acceleration of 1 rps/s requries 0.01 V output.
    slot0.kP = 60.0; // A position error of 0.2 rotations results in 12 V output.
    slot0.kI = 0.0; // No ouptut for integrated error.
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output.

    config.MotionMagic = motionMagic;
    config.Slot0 = slot0;
    intakePositionMotor.getConfigurator().apply(config);
    bottomSensor = new DigitalInput(0);
    topSensor = new DigitalInput(1);
    
  }

  public double getIntakePositionValue() {
    return intakePositionMotor.getPosition().getValueAsDouble();
  }

  public void intakeUpPosition() {
    intakePositionMotor.setControl(new MotionMagicVoltage(75.0));
  }

  public void intakeDownPosition() {
    intakePositionMotor.setControl(new MotionMagicVoltage(-90.0));
  }

  public void stopIntake() {
    intakePositionMotor.setControl(new VoltageOut(0.0));
  }

  public boolean isDeployed(){
    return bottomSensor.get();
  }

  public boolean isStored(){
    return topSensor.get();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePosTable.getEntry("Intake Position Encoder").setDouble(this.getIntakePositionValue());
  }
}
