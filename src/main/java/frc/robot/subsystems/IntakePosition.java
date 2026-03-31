// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;



public class IntakePosition extends SubsystemBase {
  public boolean ignoreUpDown = false;

  private TalonFXConfiguration config;
  private DetachedEncoderConfig absEncoderConfig;
  private TalonFX intakePositionMotor;
  private DigitalInput bottomSensor;
  private DigitalInput topSensor;
  private SplineEncoder absEncoder;
  private final double GEAR_RATIO = 27.0;


  private NetworkTable intakePosTable = NetworkTableInstance.getDefault().getTable("intake position");

  /** Creates a new IntakePosition. */
  public IntakePosition() {
    intakePositionMotor = new TalonFX(32);
    config = new TalonFXConfiguration();
    //encoder = new CANcoder(24);

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    //config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    //config.Feedback.FeedbackRemoteSensorID = 24;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

     /** MotionMagic Configs */
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)) // Reach 5 (mechanism) rotations per second cruise.
               .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(15)) // Take approximately 0.5 seconds to reach max velocity.
               .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max acceleration.

    /** PID Configs */
    Slot0Configs slot0 = config.Slot0;
    //slot0.kS = 0.25; // Add 0.25 V output to overcome static friction.
    //slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output.
    //slot0.kA = 0.01; // An acceleration of 1 rps/s requries 0.01 V output.
    slot0.kP = 15.0; // A position error of 0.2 rotations results in 12 V output. (60::FOR LUTHER TO REMEMBER)
    slot0.kI = 0.0; // No ouptut for integrated error.
    slot0.kD = 0.1; // A velocity error of 1 rps results in 0.5 V output.

    config.MotionMagic = motionMagic;
    config.Slot0 = slot0;
    intakePositionMotor.getConfigurator().apply(config);
    //intakePositionMotor.setNeutralMode(NeutralModeValue.Brake);

    bottomSensor = new DigitalInput(0);
    topSensor = new DigitalInput(1);

    absEncoder = new SplineEncoder(40);
    absEncoderConfig = new DetachedEncoderConfig();
    absEncoderConfig.inverted(true);
    //absEncoderConfig.

    //Do Encoder Config Here If Needed!
    absEncoder.configure(absEncoderConfig, ResetMode.kResetSafeParameters);
    
    seedMotorEncoder();
  }

  private void seedMotorEncoder() {

    // MAXSpline returns mechanism rotations (0–1 typically)
    //double mechanismRotations = absEncoder.getPosition();
    double mechanismRotations = absEncoder.getAngle();

    // Convert to motor rotations
    double motorRotations = mechanismRotations * GEAR_RATIO;

    intakePositionMotor.setPosition(motorRotations);
  }


  public double getIntakePositionValue() {
    //return intakePositionMotor.getPosition().getValueAsDouble();
    double motorRotations = intakePositionMotor.getPosition().getValueAsDouble();

    intakePosTable.getEntry("Intake Position Raw Motor Encoder").setDouble(motorRotations);
    intakePosTable.getEntry("Intake Position Calculated Motor Encoder").setDouble(motorRotations / GEAR_RATIO);

    return absEncoder.getAngle();
  }

  public void intakeUpPosition() {
    if (!ignoreUpDown) {
      intakePositionMotor.setControl(new MotionMagicVoltage(0.286 * GEAR_RATIO));
    }
  }

  public boolean atUpPosition() {
    if (getIntakePositionValue() >= 0.276 && getIntakePositionValue() <= 0.296) {
      return true;
    }
    return false;
  }

  public void intakeDownPosition() {
    if (!ignoreUpDown) {
      intakePositionMotor.setControl(new MotionMagicVoltage(0.0 * GEAR_RATIO));
    }
  }

  public boolean atDownPosition() {
    if (getIntakePositionValue() >= 0.0 && getIntakePositionValue() <= 0.010) {
      return true;
    }
    return false;
  }

  public void intakeBumpPosition() {
    ignoreUpDown = true;
    intakePositionMotor.setControl(new MotionMagicVoltage(0.164 * GEAR_RATIO));
  }

  public boolean atBumpPosition() {
    if (getIntakePositionValue() >= 0.154 && getIntakePositionValue() <= 0.174) {
      return true;
    }
    return false;
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
    intakePosTable.getEntry("Intake Position Absolute Encoder").setDouble(this.getIntakePositionValue());
  }
}
