
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.BooleanSupplier;

public class Indexer extends SubsystemBase {
  private TalonFX rollerMotor;
  private SparkMax indexerMotor;
  private boolean reverseMotor;
  private BooleanSupplier reverseSwitch;

  /** Creates a new Indexer. */
  public Indexer(BooleanSupplier reverseSwitch) {
    this.reverseSwitch = reverseSwitch;

    rollerMotor = new TalonFX(36);
    indexerMotor = new SparkMax(34, MotorType.kBrushed);

    reverseMotor = false;
  }

  public void runIndexer() {
    double indexerSpeed = 0.9;
    double rollerSpeed = 0.5;
    if (reverseSwitch.getAsBoolean()) {indexerSpeed *= -1; rollerSpeed *= -1;}
    indexerMotor.set(indexerSpeed);
    rollerMotor.set(rollerSpeed);
  }
  
  public void stopIndexer() {
    indexerMotor.set(0.0);
    rollerMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
