package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public final class IntakePositionCommand {
    private double lastPosisition=0;
    
    //Constructor to allow for each Command to reference the subsystem without a parameter
    private IntakePositionSubsystem intakePositionSubsystem;

    public IntakePositionCommand(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
    }
    


    //Sets the elevator and the intake arm to a specified position, which is then reached through PIDs
    public class SetIntakePositionSetpoints extends Command {

        private double liftSetpoint;
        private double pivotSetpoint;
        private Integer forLevel;

        public SetIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint, Integer forLevel) {
            this.liftSetpoint = liftSetpoint;
            this.pivotSetpoint = pivotSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setIntakePositionSetpoints(liftSetpoint, pivotSetpoint, forLevel);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.atTargetPos();
        }
    }



    //Individual setter for the elevator
    public class SetLiftSetpoint extends Command {

        private double liftSetpoint;
        private Integer forLevel;

        public SetLiftSetpoint(double liftSetpoint, Integer forLevel) {
            this.liftSetpoint = liftSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setLiftSetpoint(liftSetpoint, forLevel);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.liftAtTargetPos();
        }
    }



    //Individual setter for the intake arm
    public class SetPivotSetpoint extends Command {

        private double pivotSetpoint;
        private Integer forLevel;

        public SetPivotSetpoint(double pivotSetpoint, Integer forLevel) {
            this.pivotSetpoint = pivotSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setPivotSetpoint(pivotSetpoint, forLevel);
            if(pivotSetpoint!=Constants.IntakePositionConstants.stowPivot)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.pivotAtTargetPos();
        }
    }



    /**
     * Adjust the lift's position by running it at velocity for duration of command.
     * Continuously updates a DoubleConsumer as it runs with the current position.
     * Sets the lift's setpoint to current position when the command ends.
     */
    public class AdjustLift extends Command {

        private DoubleSupplier rateSupplier;

        public AdjustLift(DoubleSupplier rateSupplier) {
            this.rateSupplier = rateSupplier;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void execute() {
            intakePositionSubsystem.setLiftSpeed(rateSupplier.getAsDouble());
        }

        @Override
        public void end(boolean interrupted) {
            intakePositionSubsystem.setLiftSetpoint(intakePositionSubsystem.getLiftPosition(), null); // also cancels setting velocity
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }



    /**
     * Adjust the pivot's position by offsetting its position for duration of command.
     * Continuously updates a DoubleConsumer as it runs with the current position.
     * Sets the pivot's setpoint to current position when the command ends.
     */
    public class AdjustPivot extends Command {

        private DoubleSupplier rateSupplier;

        public AdjustPivot(DoubleSupplier rateSupplier) {
            this.rateSupplier = rateSupplier;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void execute() {
            intakePositionSubsystem.setPivotSpeed(rateSupplier.getAsDouble());
        }

        @Override
        public void end(boolean interrupted) {
            intakePositionSubsystem.setPivotSetpoint(intakePositionSubsystem.getPivotPosition(), null); // also cancels setting velocity
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    private boolean stopAuto0=false;
    public Command Auto0Lift() {
        return new FunctionalCommand(
            ()->intakePositionSubsystem.setLiftSpeed(0.2),
            // Start driving forward at the start of the command
            () -> {
                if(lastPosisition<=intakePositionSubsystem.getLiftPosition()){
                    stopAuto0=true;
                }
                lastPosisition=intakePositionSubsystem.getLiftPosition();
            },
            // stop lift and set 0
            interrupted -> {
                intakePositionSubsystem.zeroLift();
                intakePositionSubsystem.setLiftSpeed(0);
                stopAuto0=false;
            },

            //when stop auto0 stop it
            () -> stopAuto0,
            //require intake posistion
            intakePositionSubsystem
        );
    }

}