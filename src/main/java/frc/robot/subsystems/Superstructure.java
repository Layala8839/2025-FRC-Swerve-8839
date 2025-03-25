// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.constants.SubsystemConstants.ArmSetpoints;
import frc.robot.constants.SubsystemConstants.ElevatorSetpoints;
import frc.robot.constants.SubsystemConstants.IntakeSetpoints;


public class SuperStructure extends SubsystemBase {

  /** CoralSubsystem setpoints */
  public enum Setpoint {
    CStow,
    CLevel1,
    CLevel2,
    CLevel3,
    CLevel4,
    CLevel4test,
    CIntake,
    Cscore,
    Malgaescore,
    Malgaetravel,
    Malgaebarge,
    MalgaeLevel2,
    Malgaegroundintake,
    MalgaeLevel1,
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(frc.robot.constants.SubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(frc.robot.constants.SubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  public SparkMax intakeMotor =
      new SparkMax(frc.robot.constants.SubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kStowPosition;
  private double elevatorCurrentTarget = ElevatorSetpoints.kResting;



  public SuperStructure() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    armMotor.configure(
        Configs.Subsystem_Motors.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.Subsystem_Motors.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.Subsystem_Motors.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    
    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);
  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
    elevatorClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && elevatorMotor.getReverseLimitSwitch().isPressed()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (!elevatorMotor.getReverseLimitSwitch().isPressed()) {
      wasResetByLimit = false;
    }
  }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
                //Elevator and arm to resting position
            case CStow:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kResting;
              break;

                //Elevator to coral level 1
            case CLevel1:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;

                //Elevator to coral level 2
            case CLevel2:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;

                //Elevator to coral level 3
            case CLevel3:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;

                //Elevator to coral level 4
            case CLevel4:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;

                //Elevator to coral level 4
            case CLevel4test:
              armCurrentTarget = ArmSetpoints.kStowPosition;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;

                //Intake from the feeder station
            case CIntake:
              armCurrentTarget = ArmSetpoints.KFeederIntake;
              elevatorCurrentTarget = ElevatorSetpoints.kResting;
              break;

                //Soring Coral
            case Cscore:
              armCurrentTarget = ArmSetpoints.Kscore;
              break;

                //Algae setpoints

                //algae arm to ground intake
            case Malgaegroundintake:
              armCurrentTarget = ArmSetpoints.kballgroundintake;
              elevatorCurrentTarget = ElevatorSetpoints.kResting;
              break;

                //Elevator to algae level 1
            case MalgaeLevel1:
              armCurrentTarget = ArmSetpoints.kballLevel1;
              elevatorCurrentTarget = ElevatorSetpoints.kballLevel1;
              break;

                //Elevator to algae level 2
            case MalgaeLevel2:
              armCurrentTarget = ArmSetpoints.kballLevel2;
              elevatorCurrentTarget = ElevatorSetpoints.kballLevel2;
              break;

                //Elevator to algea level barge
            case Malgaebarge:
             armCurrentTarget = ArmSetpoints.kballbarge;
             elevatorCurrentTarget = ElevatorSetpoints.kballLevelbarge;
              break;

                //Arm to algae ball travel
            case Malgaetravel:
              armCurrentTarget = ArmSetpoints.Kballtravel;
              elevatorCurrentTarget = ElevatorSetpoints.kResting;
              break;

            case Malgaescore:
              armCurrentTarget = ArmSetpoints.kballgroundintake;
              elevatorCurrentTarget = ElevatorSetpoints.kResting;
              break;

            default:
              break;
          }
        });
  }
  /**
   * Intake commands
   */



  public Command intakeballCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(-0.1));
  }

  public Command holdIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(-0.03));
  }

  public Command ballshooterCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.8));
  }

  public Command IntakecoralCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.2));
  }

  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.15));
  }

  public Command stopIntakeCommand() {
    return this.startEnd(
      () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0));
  }

  @Override
  public void periodic() {
    moveToSetpoint();
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Coral/Arm/Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
  }
}