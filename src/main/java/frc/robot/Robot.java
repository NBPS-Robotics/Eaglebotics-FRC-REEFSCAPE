// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SensorSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  //public UsbCamera pipeCam = null;

  private Timer disabledTimer;
  Timer m_gcTimer = new Timer();

  public Robot()
  {//this runs all sensors and vision faster than the normal loop
    //vision actually updates the odometry, while sensors just update
    //thier respective values
    m_gcTimer.start();
    addPeriodic(()->{
      m_robotContainer.intakePosition.updateAll();
      SensorSubsystem.getInstance().updateAll();//all susbsystems that need pid should have the methods that
    }, 0.01,0.005);//update pid here to make sure they run as fast as possible, ONLY PID, nothing else
    instance = this;//it wont be stable then

    //pipeCam = CameraServer.startAutomaticCapture("USB Camera 0", 0);
    //pipeCam.setVideoMode(PixelFormat.kMJPEG, 192, 144, 15);
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /* public void stopCamera() {
    if (pipeCam != null) {
      pipeCam.close();
      pipeCam = null;
    }
  } */

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
//     Logger.recordMetadata("Frc 2025 tests", "tests"); // Set a metadata value

// if (isReal()) {
//     Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
//     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
//     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
// } else {
//     setUseTiming(false); // Run as fast as possible
//     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
//     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
//     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
//}

// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
//Logger.start();
    /* try {
      Runtime.getRuntime().exec("/sbin/swapon /dev/sda"); // This sucks, but competition is tomorrow.
      System.out.println("NOTICE: SWAP COMMAND FINISHED");
    } catch (IOException e) {
      System.out.println("WARNING: SWAP FIX DID NOT WORK: " + e.getMessage());
    } */
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    //PathfindingCommand.warmupCommand().schedule(); // done in SwerveSubsystem.setupPathPlanner()
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.updateSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    // if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    // {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    // }
    if (disabledTimer.advanceIfElapsed(1.0)) {
      m_robotContainer.resetOdometryFromVision();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.cancelResetOdometryFromVision();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
    //m_robotContainer.vision.updateAll();
  }

  @Override
  public void teleopInit()
  {
    m_robotContainer.cancelResetOdometryFromVision();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.stopIntakePosition();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    //m_robotContainer.vision.updateAll();
  }

  @Override
  public void testInit()
  {
    m_robotContainer.cancelResetOdometryFromVision();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.maxCurrentElevator[0]=0;
    m_robotContainer.maxCurrentElevator[1]=0;
    m_robotContainer.maxCurrentElevator[2]=0;//reset max current
    CommandScheduler.getInstance().schedule(m_robotContainer.test);//run testing routine
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
    double[] current=m_robotContainer.intakePosition.getCurrent();//save max current
    for(int i=0;i<m_robotContainer.maxCurrentElevator.length;i++){
        if(current[i]>m_robotContainer.maxCurrentElevator[i]){
          m_robotContainer.maxCurrentElevator[i]=current[i];
        }
    }
  }
  public void printCurrentTest(){//print max current from testing mode
    System.out.println("Max Current for 1. elevator motor 1, 2. elevator motor 2, 3. pivot motor");
    for(double i:m_robotContainer.maxCurrentElevator){
      System.out.print(i+" ");
    }
    SmartDashboard.putNumberArray("Max Current for 1. elevator motor 1, 2. elevator motor 2, 3. pivot motor",m_robotContainer.maxCurrentElevator);
    System.out.println("");
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

}