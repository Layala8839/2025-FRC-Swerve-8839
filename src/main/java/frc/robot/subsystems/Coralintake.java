// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;


public class Coralintake extends SubsystemBase {
  
  private boolean hasCoral, indexingCoral;

  public CoreCANrange coralSensor;
  public SparkBase intakeMotor;

  /** Creates a new CoralOuttake. 
     * @return */
    public void CoralIntake() {
    try (CANrange coralSensor = new CANrange(constants.SubsystemConstants.KCoralSensor)) {
  
      hasCoral = false;

      coralSensor.getConfigurator().apply(frc.robot.Configs.Subsystem_Motors.sensorconfig);
    }
    try (SparkMax intakemotor = new SparkMax(constants.SubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless)) {
    }

  }

  public void setCoralintake(double speed) {
    intakeMotor.set(0.1);
  }

  public void setIndexingCoral(boolean indexing) {
    this.indexingCoral = indexing;
  }

  public boolean isIndexingCoral() {
    return indexingCoral;
  }

  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public void coralToggle() {
    this.hasCoral = !hasCoral;
  }

  public boolean sensorSeesCoral() {
    return coralSensor.getIsDetected().getValue();
  }

  public BooleanSupplier sensorSeesCoralSupplier() {
    return () -> coralSensor.getIsDetected().getValue();
  }

  public boolean sensorIndexedCoral() {
    return coralSensor.getDistance().getValue().gte(constants.coralsensor.REQUIRED_CORAL_DISTANCE);
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}