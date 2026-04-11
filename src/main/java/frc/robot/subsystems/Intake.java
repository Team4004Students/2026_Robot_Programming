// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import java.util.function.BooleanSupplier;


public class Intake extends SubsystemBase {
  private TalonFXConfiguration config;
  private TalonFX intakeMotor;
  private BooleanSupplier reverseSwitch;
  public boolean intakeRunning = false;

  /** Creates a new Intake2026. */
  public Intake(BooleanSupplier reverseSwitch) {
    this.reverseSwitch = reverseSwitch;
    intakeMotor = new TalonFX(33);
    config = new TalonFXConfiguration();

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

    MotorOutputConfigs motorOutput = config.MotorOutput;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotionMagic = motionMagic;
    config.Slot0 = slot0;
    config.MotorOutput = motorOutput;
    intakeMotor.getConfigurator().apply(config);
    
  }

  public void runIntake() {
    intakeRunning = true;
    double intakeSpeed = 0.61;
    if (reverseSwitch.getAsBoolean()) {intakeSpeed *= -1;}    
    intakeMotor.set(intakeSpeed);
  }

  public void stopIntake() {
    intakeRunning = false;
    intakeMotor.setControl(new VoltageOut(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}