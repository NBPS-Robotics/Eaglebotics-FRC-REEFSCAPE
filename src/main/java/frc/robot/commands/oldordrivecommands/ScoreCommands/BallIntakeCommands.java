package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePositionConstants;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.utils.Gamepieces;
import frc.utils.IntakeState;

public final class BallIntakeCommands {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private BallIntakeSubsystem ballIntakeSubsystem;

    public BallIntakeCommands(BallIntakeSubsystem ballIntakeSubsystem) {
        this.ballIntakeSubsystem = ballIntakeSubsystem;
    }



    /** Produces a command that will toggle the intake, starting to intake if not already or stopping if so. <b>May be bugged.</b> */
    public Command toggleIntake() {
        return new ConditionalCommand(new StopIntake(), new Intake(), () -> ballIntakeSubsystem.isInState(IntakeState.INTAKE));
    }

    /** Produces a command that will toggle the outtake, starting to outtake if not already or stopping if so. <b>May be bugged.</b> */
    public Command toggleOuttake() {
        return new ConditionalCommand(new StopIntake(), new Outtake(), () -> ballIntakeSubsystem.isInState(IntakeState.OUTTAKE));
    }



    public Command getAwareOuttakeCommand(IntakePositionSubsystem intakePosition, IntakePositionCommand intakePositionCommands) {
        //Command for typical ball outtaking
        Command normalBallOuttake = new SequentialCommandGroup(
            new Outtake(),
            new StowCommand(intakePosition)
        );

        //Command for barge ball outtaking
        Command bargeOuttake = new Outtake(-IntakeConstants.kBallOuttakeBargeSpeed).withTimeout(0.25).andThen(new ParallelCommandGroup(
            new Outtake(IntakeConstants.kBallOuttakeBargeSpeed),
            Commands.runOnce(() -> intakePosition.setPivotSetpoint(IntakePositionConstants.bargePivotShove, 5))
        ));

        //C3:R1 - Ball Outtake
        return new ConditionalCommand(
            bargeOuttake,
            normalBallOuttake,
            () -> intakePosition.getPositionLevel() == 5
        );
    }



    //Turns on the ball intake
    public class Intake extends Command {
        
        public Intake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kBallIntakeSpeed, IntakeState.INTAKE);
            LEDSubsystem.getInstance().setIntake();
        }

        @Override
        public void end(boolean interrupted){
            if(!interrupted)ballIntakeSubsystem.setTargetVelocity(0.1, IntakeState.PASSIVE);
            else ballIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
            if (ballIntakeSubsystem.getHasBall()) {
                Gamepieces.gamepieceInRobot=Gamepieces.ALGAE;
                Gamepieces.activeGamepiece=Gamepieces.ALGAE;
            }
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            return ballIntakeSubsystem.getHasBall();
        }
    }



    //Turns on the ball outtake
    public class Outtake extends Command {

        private double time;
        private final double power;

        public Outtake() {
            this(Constants.IntakeConstants.kBallOuttakeSpeed);
        }

        public Outtake(double power) {
            this.power = power;
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            LEDSubsystem.getInstance().setOuttake();
            ballIntakeSubsystem.setTargetVelocity(power, IntakeState.OUTTAKE);
            time=Timer.getFPGATimestamp()+0.5;
        }

        @Override
        public void end(boolean interrupted){
            ballIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            return Timer.getFPGATimestamp()>=time;
        }
    }



    //Stops the intake/outtake
    public class StopIntake extends Command {
        
        public StopIntake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(0, IntakeState.STOP);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }
}
