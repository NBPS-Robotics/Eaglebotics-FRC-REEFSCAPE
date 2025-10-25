package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class StowCommand extends SequentialCommandGroup {
    IntakePositionCommand intakePositionCommand;
    public StowCommand(IntakePositionSubsystem intakePositionSubsystem){
        intakePositionCommand=new IntakePositionCommand(intakePositionSubsystem);
        addCommands(
            intakePositionCommand.new SetPivotSetpoint(Constants.IntakePositionConstants.stowPivot, 0),
            intakePositionCommand.new SetLiftSetpoint(Constants.IntakePositionConstants.stowLift, 0),
            new InstantCommand(()->LEDSubsystem.getInstance().setStow()),
            intakePositionCommand.Auto0Lift()
        );
    }
    
}
