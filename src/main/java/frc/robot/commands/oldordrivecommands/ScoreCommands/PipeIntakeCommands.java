package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.utils.Gamepieces;
import frc.utils.IntakeState;

public final class PipeIntakeCommands {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private PipeIntakeSubsystem pipeIntakeSubsystem;

    public PipeIntakeCommands(PipeIntakeSubsystem pipeIntakeSubsystem) {
        this.pipeIntakeSubsystem = pipeIntakeSubsystem;
    }



    /** Produces a command that will toggle the intake, starting to intake if not already or stopping if so. <b>May be bugged.</b> */
    public Command toggleIntake() {
        return new ConditionalCommand(new StopIntake(), new Intake(), () -> pipeIntakeSubsystem.isInState(IntakeState.INTAKE));
    }

    /** Produces a command that will toggle the outtake, starting to outtake if not already or stopping if so. <b>May be bugged.</b> */
    public Command toggleOuttake() {
        return new ConditionalCommand(new StopIntake(), new Outtake(), () -> pipeIntakeSubsystem.isInState(IntakeState.OUTTAKE));
    }



    public Command getAwareOuttakeCommand(IntakePositionSubsystem intakePosition, IntakePositionCommand intakePositionCommands) {
        Command outtakeNormal = new SequentialCommandGroup(
            new Outtake(),
            new StowCommand(intakePosition)
        );

        Command outtakeL1 = new SequentialCommandGroup(
            new Outtake(IntakeConstants.kPipeOuttakeL1Speed),
            new StowCommand(intakePosition)
        );

        Command outtakeL4 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    intakePositionCommands.new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe4PivotOut, null),
                    new WaitCommand(0.3)
                ),
                new Outtake(Constants.IntakeConstants.kPipeOuttakeL4Speed)
            ),
            new StowCommand(intakePosition)
        );

        return new ConditionalCommand(outtakeL4, 
                                    new ConditionalCommand(outtakeL1, outtakeNormal, () -> intakePosition.getPositionLevel() == 1),
                                    () -> intakePosition.getPositionLevel() == 4);
    }



    //Turns on the pipe intake
    public class Intake extends Command {

        public Intake(){
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            pipeIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kPipeIntakeSpeed, IntakeState.INTAKE);
            LEDSubsystem.getInstance().setIntake();
        }

        @Override
        public void end(boolean interrupted){
            pipeIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            if(pipeIntakeSubsystem.getHasPipe()){
                Gamepieces.gamepieceInRobot=Gamepieces.CORAL;
                Gamepieces.activeGamepiece=Gamepieces.CORAL;
            }
            return pipeIntakeSubsystem.getHasPipe();
        }
    }



    //Turns on the pipe outtake
    public class Outtake extends Command {

        private double time;
        private final double speed;

        public Outtake() {
            this(Constants.IntakeConstants.kPipeOuttakeSpeed);
        }

        public Outtake(double speed){
            this.speed = speed;
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            time=Timer.getFPGATimestamp()+0.5;
            pipeIntakeSubsystem.setTargetVelocity(speed, IntakeState.OUTTAKE);
            LEDSubsystem.getInstance().setOuttake();
        }

        @Override
        public void end(boolean interrupted){
            pipeIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            return Timer.getFPGATimestamp()>=time;
        }
    }



    //Stops the intake/outtake
    public class StopIntake extends Command {

        public StopIntake(){
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            pipeIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }
}
