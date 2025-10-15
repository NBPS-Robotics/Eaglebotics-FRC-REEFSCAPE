package frc.robot.commands.oldordrivecommands.TestCommands;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.oldordrivecommands.ScoreCommands.BallIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.IntakePositionCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.OpCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.PipeIntakeCommands;
import frc.robot.commands.oldordrivecommands.ScoreCommands.StowCommand;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.IntakeState;

public class TestCommand extends SequentialCommandGroup{
  private final SwerveSubsystem drive;
  private final PipeIntakeSubsystem pipeIntake;
  private final IntakePositionSubsystem intakePosition;
  private final BallIntakeSubsystem ballIntake;
  private OpCommands opCommands;
  
  

  public TestCommand(SwerveSubsystem drive, PipeIntakeSubsystem pipeIntake, IntakePositionSubsystem intakePosition,BallIntakeSubsystem ballIntake){
    this.drive=drive;
    this.ballIntake=ballIntake;
    this.intakePosition=intakePosition;
    this.pipeIntake=pipeIntake;
    opCommands=new OpCommands(intakePosition);
    addRequirements(drive,ballIntake,intakePosition,pipeIntake);
    addCommands(group0());
  }

  public Command[] group0(){
    Command[] commands={
    new StowCommand(intakePosition),//stow
    opCommands.pipeCommandGroup(3),//test pivot and elevator
    new WaitCommand(1),
    new StowCommand(intakePosition),//back to stow
    new InstantCommand(()->pipeIntake.setTargetVelocity(Constants.IntakeConstants.kPipeIntakeSpeed, IntakeState.INTAKE)),//pipe intake test
    new WaitCommand(0.5),
    new InstantCommand(()->pipeIntake.setTargetVelocity(Constants.IntakeConstants.kPipeOuttakeSpeed, IntakeState.OUTTAKE)),//outtake
    new WaitCommand(0.5),
    new InstantCommand(()->pipeIntake.setTargetVelocity(0, IntakeState.STOP)),
    new InstantCommand(()->ballIntake.setTargetVelocity(Constants.IntakeConstants.kBallIntakeSpeed, IntakeState.INTAKE)),//ball intake
    new WaitCommand(0.5),
    new InstantCommand(()->ballIntake.setTargetVelocity(Constants.IntakeConstants.kBallOuttakeSpeed, IntakeState.OUTTAKE)),//ball outtake
    new WaitCommand(0.5),
    new InstantCommand(()->ballIntake.setTargetVelocity(0, IntakeState.STOP)),
    new InstantCommand(()->Robot.getInstance().printCurrentTest())//print max current to console
    };
    return commands;
  }
}
