package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.IntakeState;

public class PipeIntakeSubsystem extends SubsystemBase {

    // Instance
    public final SparkMax m_motor;

    private IntakeState state = IntakeState.STOP;

    public PipeIntakeSubsystem() {
        // initialize motor
        m_motor = new SparkMax(Constants.IntakeConstants.kPipeMotorId, MotorType.kBrushless);

        m_motor.configure(Constants.kBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Set the target velocity of the intake and the state that the intake is now considered to be in. Pass {@code IntakeState.NONE} to avoid changing the state. */
    public void setTargetVelocity(double speed, IntakeState state) {
        m_motor.set(-speed);
        if (state != IntakeState.NONE) this.state = state;
    }

    /** Get the state of the intake (i.e. stopped, intaking, outtaking). */
    public IntakeState getState() {
        return state;
    }

    /** Whether the intake is in a particular state. Shorthand for {@code getState() == state} */
    public boolean isInState(IntakeState state) {
        return getState() == state;
    }
    
    /** Get whether the pipe sensor is triggered. Shorthand for {@code SensorSubsystem.getInstance().pipe} */
    public boolean getHasPipe(){
        return SensorSubsystem.getInstance().pipe;
    }

}
