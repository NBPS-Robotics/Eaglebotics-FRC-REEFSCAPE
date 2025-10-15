package frc.robot.commands.oldordrivecommands.TestCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotStateManegerCommand extends Command{
    OpCommands op;
    IntakePositionSubsystem elevator; 
    BallIntakeSubsystem ball;
    PipeIntakeSubsystem pipe;
    private double LastElevatorError=1000000;
    private double LastPivotError=0;
    public RobotStateManegerCommand(OpCommands op, IntakePositionSubsystem elevator, BallIntakeSubsystem ball, PipeIntakeSubsystem pipe){
        this.op=op;
        this.elevator=elevator;
        this.ball=ball;
        this.pipe=pipe;
        addRequirements();
    }

    @Override 
    public void initialize(){
        
    }

    @Override 
    public void execute(){  
        if(!elevator.liftAtTargetPos()){
            if(Math.abs(LastElevatorError)<=Math.abs(elevator.getLiftError())){
                //make the state of the robot an error state, that should signal a lift error and disable lift
            }
            LastElevatorError=elevator.getLiftError();
        }else LastElevatorError=1000000;

        if(!elevator.pivotAtTargetPos()){
            if(Math.abs(LastPivotError)<=Math.abs(elevator.getPivotError())){
                //make the state of the robot an error state, should signal a pivot error
            }
            LastPivotError=elevator.getPivotError();
        }else LastPivotError=1000000;

        if(elevator.TempError){
            //set robot state to a elevator error, lift is already disabled but make sure error state is shown
        }

        //if(VisionSubsystem.PosErrorIsOff){
        //set robot state to a vision error state, it should make robot not use functions relying on odometry
        //}
    }
}
