// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.oldordrivecommands.AutoCommands.DriveCommand;
import frc.robot.commands.oldordrivecommands.AutoCommands.WaitCommand;
import frc.robot.commands.oldordrivecommands.TestCommands.TestCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  public final VisionSubsystem vision = new VisionSubsystem(drivebase);
  public final TestCommand test=new TestCommand(drivebase);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);
  final CommandPS5Controller coDriverGamepad = new CommandPS5Controller(1);
  final CommandGenericHID buttonPanel = new CommandGenericHID(2);
  /*
   * The button IDs on the button panel follow this layout:
   * 
   * Stick panel (LED pointing forward):
   * 1 2
   * 3 4
   * Left stick: axis 0 (horizontal), axis 1 (vertical)
   * Right stick: axis 4 (horizontal), axis 5 (vertical)
   * 
   * Button panel (3 LEDs pointing forward, 1 LED pointing right):
   * 13  9  5
   * 14 10  6
   * 15 11  7
   * 16 12  8
   */

  SendableChooser<Command> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register commands for PathPlanner
    registerNamedCommands();

    configureBindingsPanel4(); // controls where driver confirms posistion selected by codriver with more automation, most of the time auto stows
    setAutoCommands();
    
    SmartDashboard.putData("Autos", autoChooser);
  }

  private void configureBindingsPanel4()
  {

    // DRIVER CONTROLS:

    //Joysticks (Default) - Drive the robot
    Command driveCommand = OpCommands.getDriveCommand(drivebase, driverGamepad);
    drivebase.setDefaultCommand(driveCommand);

    //Options - Zeros the robot heading
    driverGamepad.options().onTrue(Commands.runOnce(drivebase::zeroGyro));
  }






  public Trigger sticksInUseTrigger(CommandPS5Controller gamepad) {
    return new Trigger(() -> Math.abs(gamepad.getLeftX()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getLeftY()) > Constants.OIConstants.kDriveDeadband
                          || Math.abs(gamepad.getRightX()) > Constants.OIConstants.kDriveDeadband);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
  }

  private static double normalDegrees(double deg) {
    double mod = deg % 360.0;
    if (mod < 0) mod += 360;
    return mod;
  }
  
  public void setAutoCommands(){
    autoChooser = AutoBuilder.buildAutoChooser();

    PIDController turnController = new PIDController(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD);
    turnController.setIZone(DriveConstants.kTurningIZone);
    turnController.enableContinuousInput(0, 360);
    turnController.setSetpoint(0);

    Command pathplannerless = new SequentialCommandGroup(
      new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.k180deg))),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        drivebase.centerModulesCommand()
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(10),
        drivebase.driveCommand(() -> 0.1, () -> 0.0, () -> -turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())))
      )
    );

    Command pathplannerless2 = new SequentialCommandGroup(
      new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.k180deg))),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        drivebase.centerModulesCommand()
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        drivebase.driveCommand(() -> 0.2, () -> 0.0, () -> -turnController.calculate(normalDegrees(drivebase.getSwerveDrive().getYaw().getDegrees())))
      ),
      drivebase.driveDistanceCommand(new Translation2d(0, -0.5), 0.1),
      drivebase.driveDistanceCommand(new Translation2d(-0.1, 0), 0.1)
    );

    autoChooser.addOption("PPLESS DRIVE FORWARD", pathplannerless);
    autoChooser.addOption("PPLESS L4 SCORE", pathplannerless2);

    //AutoChooser.addOption("1Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-RedSide").getStartingPose()))));
    //AutoChooser.addOption("1Coral-Center", new ParallelCommandGroup(new PathPlannerAuto("1Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-Center").getStartingPose()))));
    //AutoChooser.addOption("1Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("1Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("1Coral-BlueSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-RedSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-RedSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-RedSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-BlueSide"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-BlueSide").getStartingPose()))));
    //AutoChooser.addOption("2Coral-BlueSide", new ParallelCommandGroup(new PathPlannerAuto("2Coral-Center"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("2Coral-Center").getStartingPose()))));
    //AutoChooser.addOption("EmptyTestAuto", new ParallelCommandGroup(new PathPlannerAuto("EmptyTestAuto"),new InstantCommand(()->drivebase.swerveDrive.resetOdometry(new PathPlannerAuto("EmptyTestAuto").getStartingPose()))));
    //AutoChooser.addOption("3Coral-RedSide", new PathPlannerAuto("3Coral-RedSide"));
  }

  public void registerNamedCommands() {
    
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public void resetOdometryFromVision() {
    vision.resetOdometry();
  }

  public void cancelResetOdometryFromVision() {
    vision.cancelResetOdometry();
  }


  public void updateSmartDashboard() {
    

    SmartDashboard.putNumber("Pigeon Oritentation", drivebase.pigeon.getAccumGyroZ().getValueAsDouble() % 360.0);

    Pose2d fieldPos = drivebase.getPose();
    SmartDashboard.putNumber("Field X Position", fieldPos.getX());
    SmartDashboard.putNumber("Field Y Position", fieldPos.getY());
    SmartDashboard.putNumber("Field Heading", fieldPos.getRotation().getDegrees());

    int i=1;
    for (SwerveModule module : drivebase.swerveDrive.swerveDriveConfiguration.modules) {
      SmartDashboard.putNumber("module absolute "+i, module.getAbsoluteEncoder().getAbsolutePosition());
      SmartDashboard.putNumber("module angle "+i, module.getState().angle.getDegrees());
      SmartDashboard.putNumber("module offset "+i,Math.abs(module.getAbsoluteEncoder().getAbsolutePosition()-module.getState().angle.getDegrees()));
      i++;
    }
    
    /* double goToStow = SmartDashboard.getNumber("Go to stow", 0);
    if (goToStow > 0.001) dashboardStowCommand.schedule();
    SmartDashboard.putNumber("Go to stow", 0);
    SmartDashboard.putBoolean("Will go stow?", dashboardStowCommand.isScheduled()); */

    SmartDashboard.updateValues();
  }

  //private Command dashboardStowCommand = opCommands.getStowParallelCommand();
}
