package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;

public class HangSubsystem extends SubsystemBase {

    // Instance
    public final SparkMax m_twistMotor;
    public final SparkMax m_gripMotor;
    public final Servo servo1;
    public final Servo servo2;


    public HangSubsystem() {
        // initialize motor
        m_twistMotor = new SparkMax(Constants.HangConstants.kHangTwistMotorId, MotorType.kBrushless);
        m_gripMotor = new SparkMax(Constants.HangConstants.kHangGripMotorId, MotorType.kBrushless);

        m_twistMotor.configure(Constants.kBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_gripMotor.configure(Constants.kBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        servo1 = new Servo(Constants.HangConstants.kServo1Channel);
        servo2 = new Servo(Constants.HangConstants.kServo2Channel);
    }

    public void setServoPositions(double position){
        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    public void setTwistPower(double power) {
        m_twistMotor.set(power);
    }

    public void setGripPower(double power) {
        m_gripMotor.set(power);
    }

}
