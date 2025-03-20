// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.AlgaeSubsystem.Setpoint2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.Other.Sensor.CANRange;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Controller Name
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
    private final AlgaeSubsystem m_algaeSubSystem = new AlgaeSubsystem();


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {
        SmartDashboard.putData("Run Selected Auto", new InstantCommand(() -> {
                Command autoCommand = autoChooser.getSelected();
                if (autoCommand != null) { autoCommand.schedule(); 
                        } else 
                                {System.out.println("No Auto Selected");}}));

        //subsystem initialization
       CoralSubsystem m_coralSubSystem = new CoralSubsystem();
       AlgaeSubsystem m_algaeSubSystem = new AlgaeSubsystem();
        //Register Named Commands
        //Coral
        NamedCommands.registerCommand("Coral_Klevel1", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
        NamedCommands.registerCommand("Coral_Klevel2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
        NamedCommands.registerCommand("Coral_Klevel3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
        NamedCommands.registerCommand("Coral_Klevel4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
        NamedCommands.registerCommand("Intake_Coral_Feeder", m_coralSubSystem.setSetpointCommand(Setpoint.KIntake));
        NamedCommands.registerCommand("score", m_coralSubSystem.setSetpointCommand(Setpoint.Kscore));
        NamedCommands.registerCommand("KStow", m_coralSubSystem.setSetpointCommand(Setpoint.kStow));

        //Algae
        NamedCommands.registerCommand("Ball_Level_1", m_algaeSubSystem.setSetpointCommand(Setpoint2.kballLevel1));
        NamedCommands.registerCommand("Ball_Level_2", m_algaeSubSystem.setSetpointCommand(Setpoint2.kballLevel2));
        NamedCommands.registerCommand("Ball_Barge", m_algaeSubSystem.setSetpointCommand(Setpoint2.kballbarge));
        NamedCommands.registerCommand("Ball_Score", m_algaeSubSystem.setSetpointCommand(Setpoint2.kballscore));
        NamedCommands.registerCommand("Ball_Ground_Intake", m_algaeSubSystem.setSetpointCommand(Setpoint2.Kballgroundintake));
        NamedCommands.registerCommand("Ball_Intake", m_algaeSubSystem.setSetpointCommand(Setpoint2.Kballintake));
            //Do all after initialization
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/3) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed/3) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate/3) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.start().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       /*/ joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); */

/**reset the field-centric heading on left bumper press**/
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

//*************************************************************************************************************/
//Coral Intake

                //Joystick Left Trigger Runs Intake Method
/*      joystick.leftTrigger().whileTrue(m_coralSubSystem.reverseIntakeCommand().until(() -> CANRange.getIsDetected))
                .whileFalse(m_coralSubSystem.runIntakeCommand());//.whileTruerunIntakeCommand
        joystick.leftTrigger().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.KIntake))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));
                
*/

//I think this would worth for detecting the coral
        joystick.leftTrigger().onTrue(m_coralSubSystem.reverseIntakeCommand().until(() -> CANRange.getIsDetected));
        joystick.leftTrigger().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.KIntake).until(() -> CANRange.getIsDetected));


        joystick.leftTrigger().onTrue(m_coralSubSystem.reverseIntakeCommand().until(() -> CANRange.getIsDetected));
        joystick.leftTrigger().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.KIntake).until(() -> CANRange.getIsDetected));

        joystick.leftTrigger().onFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));
//Elevator Button Bindings

                //Joystick "A" Runs elevator and arm to Level 1
        joystick.a().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));

                //Joystick "B" Runs elevator and arm to Level 2
        joystick.b().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));

                //Joystick "X" Runs elevator and arm to Level 3
        joystick.x().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));

                //Joystick "Y" Runs elevator and arm to Level 4
        joystick.y().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));  
        
// Score System

                //Joystick "RightTrigger" Runs Intake Command
        joystick2.rightTrigger().whileTrue(m_coralSubSystem.reverseIntakeCommand())
                .whileFalse(m_coralSubSystem.runIntakeCommand());
        joystick2.rightTrigger().whileTrue(m_coralSubSystem.setSetpointCommand(Setpoint.Kscore))
                .whileFalse(m_coralSubSystem.setSetpointCommand(Setpoint.kStow));

//Algea ball intake

                //Joystock Right Bumer on controller 2 moves arm to Ball Ground Intake
        joystick2.rightBumper().whileTrue(m_algaeSubSystem.setSetpointCommand(Setpoint2.Kballgroundintake))
                .whileFalse(m_algaeSubSystem.setSetpointCommand(Setpoint2.kStow));

//Ball Reef pick up
    }


        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }
    }
