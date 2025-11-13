package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem with two motors under closed-loop velocity control (software PID).
 * Units: RPM for setpoints and measurements (RelativeEncoder.getVelocity() reports RPM).
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final PIDController leftVelocityPID;
  private final PIDController rightVelocityPID;

  private volatile double targetRPM = 0.0;

  /**
   * Create a shooter with two brushless SparkMax motors using velocity PID control.
   * @param leftMotorId CAN ID for the left motor
   * @param rightMotorId CAN ID for the right motor
   */
  public ShooterSubsystem(int leftMotorId, int rightMotorId) {
    leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    // Default PID gains (tune as needed)
    leftVelocityPID = new PIDController(0.0008, 0.0, 0.0);
    rightVelocityPID = new PIDController(0.0008, 0.0, 0.0);

    // Reasonable velocity tolerance in RPM; adjust per mechanism
    leftVelocityPID.setTolerance(50.0);
    rightVelocityPID.setTolerance(50.0);

    // Ensure both motors spin the same direction by default; invert one if needed externally
    leftMotor.setInverted(false);
    rightMotor.setInverted(false);
  }

  /**
   * Set both shooter motors to the desired velocity in RPM.
   * @param rpm desired wheel RPM
   */
  public void setSpeedRPM(double rpm) {
    targetRPM = rpm;
  }

  /**
   * Stop the shooter and reset controllers.
   */
  public void stop() {
    targetRPM = 0.0;
    leftVelocityPID.reset();
    rightVelocityPID.reset();
    leftMotor.set(0.0);
    rightMotor.set(0.0);
  }

  /**
   * Configure PID gains for both sides.
   */
  public void setPID(double kP, double kI, double kD) {
    leftVelocityPID.setPID(kP, kI, kD);
    rightVelocityPID.setPID(kP, kI, kD);
  }

  /**
   * Invert one or both motors if your wiring causes opposite spin directions.
   */
  public void setInverted(boolean leftInverted, boolean rightInverted) {
    leftMotor.setInverted(leftInverted);
    rightMotor.setInverted(rightInverted);
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getLeftRPM() {
    return leftEncoder.getVelocity();
  }

  public double getRightRPM() {
    return rightEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    return leftVelocityPID.atSetpoint() && rightVelocityPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // Calculate duty cycle from PID (clamped to [-1, 1])
    double leftOutput = leftVelocityPID.calculate(getLeftRPM(), targetRPM);
    double rightOutput = rightVelocityPID.calculate(getRightRPM(), targetRPM);
    leftOutput = Math.max(-1.0, Math.min(1.0, leftOutput));
    rightOutput = Math.max(-1.0, Math.min(1.0, rightOutput));

    leftMotor.set(leftOutput);
    rightMotor.set(rightOutput);

    SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
    SmartDashboard.putNumber("Shooter/Left RPM", getLeftRPM());
    SmartDashboard.putNumber("Shooter/Right RPM", getRightRPM());
    SmartDashboard.putBoolean("Shooter/At Setpoint", atSetpoint());
  }
}
