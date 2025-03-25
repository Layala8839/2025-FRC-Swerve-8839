// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Setpoint;
//import frc.robot.subsystems.Other.Sensor.CANRange;

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
    private final CommandXboxController joystick1 = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    //Subsystem commands {setpoints}
    private final SuperStructure m_superstructure = new SuperStructure();
    //private final AlgaeSubsystem m_superstructure = new AlgaeSubsystem();


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
   ////////// private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {/* 
        SmartDashboard.putData("Run Selected Auto", new InstantCommand(() -> {
                Command autoCommand = autoChooser.getSelected();
                if (autoCommand != null) { autoCommand.schedule(); 
                        } else 
                                {System.out.println("No Auto Selected");}}));*/

        //subsystem initialization
       /*SuperStructure m_superstructure = new SuperStructure();*/
       //AlgaeSubsystem m_superstructure = new AlgaeSubsystem();
        //Register Named Commands
        //Coral
        /*
        NamedCommands.registerCommand("Coral_Klevel1", 
                m_superstructure.setSetpointCommand(Setpoint.kLevel1));

        NamedCommands.registerCommand("Coral_Klevel2", 
                m_superstructure.setSetpointCommand(Setpoint.kLevel2));

        NamedCommands.registerCommand("Coral_Klevel3", 
                m_superstructure.setSetpointCommand(Setpoint.kLevel3));

        NamedCommands.registerCommand("Coral_Klevel4", 
                m_superstructure.setSetpointCommand(Setpoint.kLevel4));

        NamedCommands.registerCommand("Intake_Coral_Feeder", 
                m_superstructure.setSetpointCommand(Setpoint.KIntake));

        NamedCommands.registerCommand("Coral_Score", 
                m_superstructure.setSetpointCommand(Setpoint.Kscore));

        NamedCommands.registerCommand("KStow", 
                m_superstructure.setSetpointCommand(Setpoint.kStow));

        NamedCommands.registerCommand("Intake", 
                m_superstructure.runIntakeCommand().until(() -> 
                CANRange.getIsDetected));*/


        //Algae
        /* 
        NamedCommands.registerCommand("Ball_Level_1", 
        m_superstructure.setSetpointCommand(Setpoint.kalgaeLevel1));

        NamedCommands.registerCommand("Ball_Level_2", 
        m_superstructure.setSetpointCommand(Setpoint.kballLevel2));

        NamedCommands.registerCommand("Ball_Barge", 
        m_superstructure.setSetpointCommand(Setpoint.kballbarge));

        NamedCommands.registerCommand("Ball_Score", 
        m_superstructure.setSetpointCommand(Setpoint.kballscore));

        NamedCommands.registerCommand("Ball_Ground_Intake", 
        m_superstructure.setSetpointCommand(Setpoint.Kballgroundintake));

        NamedCommands.registerCommand("Ball_Intake", 
        m_superstructure.setSetpointCommand(Setpoint.Kballintake));
        //Do all after initialization
        */
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed/3) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick1.getLeftX() * MaxSpeed/3) // Drive left with negative X (left)
                    .withRotationalRate(-joystick1.getRightX() * MaxAngularRate/3) // Drive counterclockwise with negative X (left)
            )
        );

        joystick1.start().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick1.povDown().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))));
              
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       /*/ joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); */

/**reset the field-centric heading on left bumper press**/
        joystick1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


/********************************************Joystick 1****************************************************************/

/******************************************Coral Commands***********************************************************************/

                //I think this would worth for detecting the coral
        joystick1.rightBumper().onTrue(m_superstructure.IntakecoralCommand());
        
                //.onFalse(m_superstructure.stopIntakeCommand());
        joystick1.rightBumper().onTrue(m_superstructure.setSetpointCommand(Setpoint.CIntake))
                .onFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));

                //Joystick "A" Runs elevator and arm to Level 1
        joystick1.a().whileTrue(m_superstructure.setSetpointCommand(Setpoint.CLevel1))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));

                //Joystick "B" Runs elevator and arm to Level 2
        joystick1.b().whileTrue(m_superstructure.setSetpointCommand(Setpoint.CLevel2))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));

                //Joystick "X" Runs elevator and arm to Level 3
        joystick1.x().whileTrue(m_superstructure.setSetpointCommand(Setpoint.CLevel3))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));

                //Joystick "Y" Runs elevator and arm to Level 4
        joystick1.y().whileTrue(m_superstructure.setSetpointCommand(Setpoint.CLevel4test))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));

                //Score commands while holding the level of hight button
        joystick1.rightTrigger().whileTrue(m_superstructure.reverseIntakeCommand())
                .whileFalse(m_superstructure.IntakecoralCommand());
        joystick1.rightTrigger().whileTrue(m_superstructure.setSetpointCommand(Setpoint.Cscore))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));
        joystick1.leftTrigger().whileTrue(m_superstructure.reverseIntakeCommand())
                .whileFalse(m_superstructure.IntakecoralCommand());
        joystick1.leftBumper().whileTrue(m_superstructure.setSetpointCommand(Setpoint.Cscore))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));


/******************************************** Controller 2 ****************************************************************/

/************************************************Algae****************************************************************/

                //Algae ground intake
        joystick2.x().whileTrue(m_superstructure.setSetpointCommand(Setpoint.Malgaegroundintake))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.Malgaetravel));

                //algae reef level 1 position
        joystick2.a().whileTrue(m_superstructure.setSetpointCommand(Setpoint.MalgaeLevel1))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.Malgaetravel));

                //Algae reef level 2 position
        joystick2.b().whileTrue(m_superstructure.setSetpointCommand(Setpoint.MalgaeLevel2))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.Malgaetravel));

                //Algae Scor position
        joystick2.y().whileTrue(m_superstructure.setSetpointCommand(Setpoint.Malgaescore))
                .whileFalse(m_superstructure.setSetpointCommand(Setpoint.CStow));
        
                //Algae intake command
        joystick2.rightTrigger().whileTrue(m_superstructure.intakeballCommand())
                .whileFalse(m_superstructure.holdIntakeCommand());

                //Algae Score Command
        joystick2.leftTrigger().whileTrue(m_superstructure.reverseIntakeCommand())
                .whileFalse(m_superstructure.stopIntakeCommand());
    }

        //public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                ////////return autoChooser.getSelected();}
    }
