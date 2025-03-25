// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

@Logged
public class Coralintake extends SubsystemBase {
  TalonFX outtakeMotor;
  CANrange coralSensor;
  private boolean hasCoral, indexingCoral;

  /** Creates a new CoralOuttake. 
     * @return */
    public void CoralIntake() {
    coralSensor = new CANrange(constants.SubsystemConstants.KCoralSensor);

    hasCoral = false;

    coralSensor.getConfigurator().apply(frc.robot.Configs.Subsystem_Motors.sensorconfig);
  }

  public void setCoralOuttake(double speed) {
    outtakeMotor.set(speed);
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