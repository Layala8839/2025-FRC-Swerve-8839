package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Csetpoint;
import frc.robot.subsystems.IntakeCommands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CoralReefScore {

    private final SuperStructure m_superstructure = new SuperStructure();
    private final IntakeCommands m_intakecommands = new IntakeCommands();

    public Command ReefLevel4Command() {
        return this.commandsequence(
            m_superstructure.setSetpointCommand(Csetpoint.CLevel4),
            m_superstructure.setSetpointCommand(Csetpoint.Cscore),
            m_intakecommands.IntakecoralCommand().alongWith( m_superstructure.
                setSetpointCommand(Csetpoint.Cscore)).andThen(new WaitCommand(0.5)),
            m_superstructure.setSetpointCommand(Csetpoint.CStow)
            );
        }
    public Command reefLevel3Command() {
        return this.commandsequence(
            m_superstructure.setSetpointCommand(Csetpoint.CLevel3),
            m_superstructure.setSetpointCommand(Csetpoint.Cscore),
            m_intakecommands.IntakecoralCommand().alongWith( m_superstructure.
                setSetpointCommand(Csetpoint.Cscore)).andThen(new WaitCommand(0.5)),
            m_superstructure.setSetpointCommand(Csetpoint.CStow)
            );
    }
    public Command reefLevel2Command() {
        return this.commandsequence(
            m_superstructure.setSetpointCommand(Csetpoint.CLevel2),
            m_superstructure.setSetpointCommand(Csetpoint.Cscore),
            m_intakecommands.IntakecoralCommand().alongWith( m_superstructure.
                setSetpointCommand(Csetpoint.Cscore)).andThen(new WaitCommand(0.5)),
            m_superstructure.setSetpointCommand(Csetpoint.CStow)
            );
    }
    public Command reefLevel1Command() {
        return this.commandsequence(
            m_superstructure.setSetpointCommand(Csetpoint.CLevel1),
            m_superstructure.setSetpointCommand(Csetpoint.Cscore),
            m_intakecommands.IntakecoralCommand().alongWith( m_superstructure.
                setSetpointCommand(Csetpoint.Cscore)).andThen(new WaitCommand(0.5)),
            m_superstructure.setSetpointCommand(Csetpoint.CStow)
                );
    }

    public static void main(String[] args) {
        CoralReefScore coralReefCommands = new CoralReefScore();
        Command command = 
            coralReefCommands.ReefLevel4Command();
            command.execute();
    }
        
            private Command commandsequence(Command... commands) {
                // TODO Implement the logic to handle the sequence of commands
                throw new UnsupportedOperationException("Unimplemented method 'commandsequence'");
            }


   // public Command KL4() [
    //    return this.startEnd(
    //        Subset
     //   )}

    
}
