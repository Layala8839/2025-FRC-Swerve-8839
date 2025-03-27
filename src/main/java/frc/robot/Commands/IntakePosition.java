package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeDetect;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Csetpoint;



public class IntakePosition {

    private final SuperStructure m_superstructure = new SuperStructure();
    private final IntakeDetect m_intakedetect = new IntakeDetect();

    public Command IntakeCoral() {
        return this.commandsequence(
            m_intakedetect.runIntakeUntilDetected().alongWith( m_superstructure.
                setSetpointCommand(Csetpoint.Cscore)).andThen(new WaitCommand(0.05)),
            m_superstructure.setSetpointCommand(Csetpoint.CStow)
            );
        }

    private Command commandsequence(SequentialCommandGroup andThen, Command setSetpointCommand) {
        throw new UnsupportedOperationException("Intaked Coral");
    }
    
}
