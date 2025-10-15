package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HangConstants;
import frc.robot.subsystems.HangSubsystem;

public final class HangCommands {

    private HangSubsystem hangSubsystem;
    private IntakePositionCommand intakePositionCommands;

    private int stage = 0;

    //Constructor to allow for each Command to reference the subsystem without a parameter
    public HangCommands(HangSubsystem hangSubsystem, IntakePositionCommand intakePositionCommands) {
        this.hangSubsystem = hangSubsystem;
        this.intakePositionCommands = intakePositionCommands;
    }


    
    public Command multiStageHangCommand() {
        return new SequentialCommandGroup(new SelectCommand<>(new HashMap<>(){{
            put(0, new Grip(HangConstants.kHangGripPower));
            put(1, activateCommand());
            put(2, new Twist(HangConstants.kHangTwistPower));
        }}, () -> stage),
        Commands.runOnce(() -> stage = Math.min(2, stage + 1)));
    }


    public Command activateCommand() {
        return new ParallelCommandGroup(
            Commands.runOnce(() -> hangSubsystem.setServoPositions(HangConstants.kHangUnlockPos), hangSubsystem),
            intakePositionCommands.new SetIntakePositionSetpoints(HangConstants.kHangLiftPos, HangConstants.kHangPivotPos, 0)
        );
    }



    //Positions the hang for grabbing onto the thing I forgot the name of
    public class Grip extends Command {

        private final double targetPower;

        public Grip(double power){
            this.targetPower = power;
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            hangSubsystem.setGripPower(targetPower);
        }

        @Override
        public void end(boolean interrupted) {
            hangSubsystem.setGripPower(0);
        }

        @Override 
        public boolean isFinished() { 
            return false;
        }
    }



    //Twists the hang
    public class Twist extends Command {

        private final double targetPower;

        public Twist(double power){
            this.targetPower = power;
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            hangSubsystem.setTwistPower(targetPower);
        }

        @Override
        public void end(boolean interrupted) {
            hangSubsystem.setTwistPower(0);
        }

        @Override 
        public boolean isFinished() { 
            return false;
        }
    }
}
