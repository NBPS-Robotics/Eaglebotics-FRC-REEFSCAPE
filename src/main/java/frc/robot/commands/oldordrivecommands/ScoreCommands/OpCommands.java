package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SensorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OpCommands {

    // STATIC

    public static Command getDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        /* Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverGamepad.getLeftY(), Constants.OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(driverGamepad.getLeftX(), Constants.OIConstants.kDriveDeadband),
            () -> driverGamepad.getRightX(),
            () -> driverGamepad.getRightY()); */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> -gamepad.getRightX(),
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);

        return driveFieldOrientedAnglularVelocity;


    }

    /* public static Command getSlowDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // same as above but half as fast
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY() * Constants.DriveConstants.speedFactor,
            () -> -gamepad.getLeftX() * Constants.DriveConstants.speedFactor,
            () -> -gamepad.getRightX() * Constants.DriveConstants.speedFactor,
            OIConstants.kDriveDeadband, OIConstants.kDriveDeadband);

        return driveFieldOrientedAnglularVelocity;

    
    } */

    /**
     * A command that will strafe from the gamepad's left stick and turn to face an angle specified in degrees: [0, 360).
     * <p><b>Might cause a memory leak.</b> Whoops!</p>
     */
    public static Command getAutoTurnDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad, double degrees) {
        PIDController turnController = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
        turnController.setIZone(DriveConstants.kTurningIZone);
        turnController.enableContinuousInput(0, 360);
        turnController.setSetpoint(degrees);

        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> -gamepad.getLeftY(),
            () -> -gamepad.getLeftX(),
            () -> turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())),
            OIConstants.kDriveDeadband, 0);
            turnController.close();
        return driveFieldOrientedAnglularVelocity;
    
    }

    private static double normalDegrees(double deg) {
        double mod = deg % 360.0;
        if (mod < 0) mod += 360;
        return mod;
    }

    public static Command getTemporarySlowSpeedCommand(SwerveSubsystem swerve) {
        return new Command() {
            @Override
            public void initialize() {
                swerve.driveMultiplier = DriveConstants.speedSlow;
            }

            @Override
            public void end(boolean interrupted) {
                swerve.driveMultiplier = DriveConstants.speedFull;
            }
        };
    }



    // INSTANCE


    //Constructor to allow for each Command to reference the subsystem without a parameter
    private final IntakePositionSubsystem intakePositionSubsystem;
    private final IntakePositionCommand intakePositionCommand;

    public OpCommands(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
        intakePositionCommand = new IntakePositionCommand(intakePositionSubsystem);
    }


    //Individual commands for each set position needed for the four levels of the ball and pipe

    /**
     * Move lift and pivot simultaneously to the stow position.
     */
    public ParallelCommandGroup getStowParallelCommand() {
        return new ParallelCommandGroup(intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.IntakePositionConstants.stowLift, Constants.IntakePositionConstants.stowPivot, 0),
            new InstantCommand(()->LEDSubsystem.getInstance().setStow())
            );
    }

    public Command quickStowPivot(Integer forLevel) {
        return Commands.runOnce(() -> intakePositionSubsystem.setPivotSetpoint(Constants.IntakePositionConstants.stowPivot, forLevel));
    }

    public Command quickStowLift(Integer forLevel) {
        return Commands.runOnce(() -> intakePositionSubsystem.setLiftSetpoint(Constants.IntakePositionConstants.stowLift, forLevel));
    }

    /**
     * Signal the lift and pivot to move to stow and end immediately.
     * Other intake position commands wait for them to make it to the destination. This one does not.
     */
    public Command quickStowCommand() {
        return new ParallelCommandGroup(
            quickStowLift(0),
            quickStowPivot(0)
        );
    }

    /**
     * Signal the lift and pivot to move to the pipe intake position and end immediately.
     * Other intake position commands wait for them to make it to the destination. This one does not.
     */
    public Command quickPipeIntakeCommand() {
        return Commands.runOnce(() -> intakePositionSubsystem.setIntakePositionSetpoints(
            Constants.OpConstantsForPipe.PipeIntakeLift, Constants.OpConstantsForPipe.PipeIntakePivot, 0));
    }

    /**
     * Move lift and pivot simultaneously to the ball ground intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball1Lift, Constants.OpConstantsForBall.Ball1Pivot, 1);
    }

    /**
     * Move lift and pivot simultaneously to the ball processor scoring position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball2Lift, Constants.OpConstantsForBall.Ball2Pivot, 2);
    }

    /**
     * Move lift and pivot simultaneously to the ball level 2 intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball3Lift, Constants.OpConstantsForBall.Ball3Pivot, 3);
    }

    /**
     * Move lift and pivot simultaneously to the ball level 3 intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball4Lift, Constants.OpConstantsForBall.Ball4Pivot, 4);
    }

    /** Move lift and pivot simultaneously to the pipe intake position. */
    public Command getPipeIntakeCommand() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.PipeIntakeLift, Constants.OpConstantsForPipe.PipeIntakePivot, 0);
    }

    /** Same as getPipeIntakeCommand(), but afterwards runs the pipe intake and then stows. */
    public SequentialCommandGroup getPipeIntakeFullCommand(PipeIntakeCommands pipeIntakeCommands) {
        return new SequentialCommandGroup(
            getPipeIntakeCommand(),
            pipeIntakeCommands. new Intake(),
            new StowCommand(intakePositionSubsystem)
        );
    }
    //TODO: test if needed
    /** theoretically should bring pivot back in range, if it passes 0*/
    public SequentialCommandGroup getPivotRangeFixCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(()->intakePositionSubsystem.setPivotSpeed(-0.75)),
            new WaitCommand(0.5),
            new InstantCommand(()->intakePositionSubsystem.setPivotSpeed(0))
        );
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 1 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe1Lift, Constants.OpConstantsForPipe.Pipe1Pivot, 1);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 2 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe2Lift, Constants.OpConstantsForPipe.Pipe2Pivot, 1);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 3 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe3Lift, Constants.OpConstantsForPipe.Pipe3Pivot, 3);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 4 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe4Lift, Constants.OpConstantsForPipe.Pipe4Pivot, 4);
    }


    


    //basic command groups

    /**
     * Move lift and pivot in order to the barge shooting position.
     */
    public SequentialCommandGroup bargeShootCommandGroup() {
        return new SequentialCommandGroup(
            //new InstantCommand(()->LEDSubsystem.getInstance().setBarge()),
            intakePositionCommand.new SetIntakePositionSetpoints(Constants.IntakePositionConstants.bargeLift, Constants.IntakePositionConstants.bargePivotTravel, 5),
            new InstantCommand(() -> intakePositionSubsystem.setPivotSetpoint(Constants.IntakePositionConstants.bargePivot, 5))
        );
    }

    /**
     * Go to high algae intake position and sweep down to intake an algae on either L3 or L2.
     */
    public Command ballSweepCommand() {
        return new SequentialCommandGroup(
            ballCommandGroup(4),
            new ParallelRaceGroup(
                getBall3Command(),
                new WaitUntilCommand(() -> SensorSubsystem.getInstance().ball)
            )
        );
    }

    /**
     * Move first the lift, then the pivot to a set position for pipes.
     * @param level The level to score at (within 1-4, otherwise will go to stow position)
     */
    public SequentialCommandGroup pipeCommandGroup(int level){
        SequentialCommandGroup command;
        switch (level) {
            case 1:
                command= new SequentialCommandGroup(
                    quickStowPivot(1),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe1Lift, 1),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe1Pivot, 1)
                );
                break;
            case 2:
                command= new SequentialCommandGroup(
                    quickStowPivot(2),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe2Lift, 2),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe2Pivot, 2)
                );
                
                break;
            case 3:
                 command= new SequentialCommandGroup(
                    quickStowPivot(3),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe3Lift, 3),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe3Pivot, 3)
                );
                
                break;
            case 4:
                command= new SequentialCommandGroup(
                    quickStowPivot(4),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe4Lift, 4),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe4Pivot, 4)
                );
                
                break;
            default:
                command=new StowCommand(intakePositionSubsystem);
                break;
            
        }
        return command;
    }

    /**
     * Move first the lift, then the pivot to a set position for balls.
     * <p>Levels:</p>
     * <p>1 - Ground intake</p>
     * <p>2 - Proccessor</p>
     * <p>3 - Level 2 intake</p>
     * <p>4 - Level 3 intake</p>
     * <p>Anything else - Stow position</p>
     */
    public SequentialCommandGroup ballCommandGroup(int level){
        SequentialCommandGroup command;
        switch (level) {
            case 1:
                command= new SequentialCommandGroup(
                    quickStowPivot(1),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball1Lift, 1),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball1Pivot, 1)
                );
                break;
            case 2:
                command= new SequentialCommandGroup(
                    quickStowPivot(2),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball2Lift, 2),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball2Pivot, 2)
                );
                
                break;
            case 3:
                 command= new SequentialCommandGroup(
                    quickStowPivot(3),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball3Lift, 3),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball3Pivot, 3)
                );
                
                break;
            case 4:
                command= new SequentialCommandGroup(
                    quickStowPivot(4),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball4Lift, 4),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball4Pivot, 4)
                );
                
                break;
            case 5:
                command= new SequentialCommandGroup(
                    quickStowPivot(5),
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball5Lift,5),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball5Pivot,5)
                );
        
                break;
            default:
                command= new StowCommand(intakePositionSubsystem);
                break;
            
        }
        return command;
    }

}