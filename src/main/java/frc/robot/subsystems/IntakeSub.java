package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.constants.SubsystemConstants.IntakeSetpoints;

public class IntakeSub extends SubsystemBase {
    // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
    // controller like above.
    public SparkMax intakeMotor =
        new SparkMax(frc.robot.constants.SubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

    public IntakeSub() {
        //Apply the appropriate configurations to the SPARKs.
        intakeMotor.configure(
                Configs.Subsystem_Motors.intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    //******************//
    //Coral Cancoder Setup
    public CANrange CANrange = 
        new CANrange(frc.robot.constants.SubsystemConstants.KCoralSensor);

    public static final CANrangeConfiguration CANrangeconfig = new CANrangeConfiguration();

    static {
        CANrangeconfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    }

    public boolean hasCoral, nocoral;

    public StatusSignal<Boolean> getIsDetected(boolean refresh) {
        return CANrange.getIsDetected(refresh);
    }

    //*****************//
    //Intake Commands 

    @Override
    public void periodic() {
        // Display subsystem values
        SmartDashboard.putNumber("Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
        
        // Check if the CANRange sensor detects a game piece
        boolean isDetected = getIsDetected(true).getValue();
        SmartDashboard.putBoolean("CANRange Detected", isDetected);
        SmartDashboard.putBoolean("GamePieceDetected", isDetected);

        if (isDetected) {
            // Stop the intake motor if a game piece is detected
            setIntakePower(0);
        }
    }

    public Command runIntakeUntilDetected() {
        return this.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kForward),
            () -> this.setIntakePower(0.2)
        ).until(() -> getIsDetected(true).getValue());
    }

    
}
