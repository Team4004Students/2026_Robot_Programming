// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;


public class Shooter extends SubsystemBase {
  private TalonFXConfiguration config;
  private TalonFX rightShooterMotor;
  private TalonFX leftShooterMotor;
  private NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("shooter");
  private BooleanSupplier reverseSwitch;

  /** Creates a new Shooter. */
  public Shooter(BooleanSupplier reverseSwitch) {
    this.reverseSwitch = reverseSwitch;
    rightShooterMotor = new TalonFX(30);
    leftShooterMotor = new TalonFX(31);
    config = new TalonFXConfiguration();

     /** MotionMagic Configs */
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(5)) // Reach 5 (mechanism) rotations per second cruise.
               .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max velocity.
               .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100)); // Take approximately 0.1 seconds to reach max acceleration.

    MotorOutputConfigs motorOutput = config.MotorOutput;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;
               
    /** PID Configs */
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction.
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output.
    slot0.kA = 0.01; // An acceleration of 1 rps/s requries 0.01 V output.
    slot0.kP = 60.0; // A position error of 0.2 rotations results in 12 V output.
    slot0.kI = 0.0; // No ouptut for integrated error.
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output.

    config.MotionMagic = motionMagic;
    config.MotorOutput = motorOutput;
    config.Slot0 = slot0;
    rightShooterMotor.getConfigurator().apply(config);
    leftShooterMotor.getConfigurator().apply(config);

  }

  public void runShooter() {
    //rightShooterMotor.setControl(new MotionMagicVelocityVoltage(8));
    double shooterSpeed = 0.75;
    if (reverseSwitch.getAsBoolean()) {shooterSpeed *= -1;}
    rightShooterMotor.set(shooterSpeed);
    leftShooterMotor.setControl(new Follower(30, MotorAlignmentValue.Opposed));
  }

  public void stopShooter() {
    rightShooterMotor.setControl(new VoltageOut(0.0));
    leftShooterMotor.setControl(new Follower(30, MotorAlignmentValue.Opposed));
  }

  public boolean atSpeed() {
    if (rightShooterMotor.getVelocity().getValueAsDouble() >= 70) {
      return true;
    }
    else {
      return false;
    }
  }
  
  @Override
  public void periodic() {
    shooterTable.getEntry("Shooter RPS").setDouble(rightShooterMotor.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
