package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakePositionConstants;
import frc.robot.commands.oldordrivecommands.AutoCommands.WaitCommand;
import frc.robot.commands.oldordrivecommands.ScoreCommands.StowCommand;

public class IntakePositionSubsystem extends SubsystemBase {
    //private double lastPosisition=0;
    public static enum Posistions {
        L4,
        L3,
        L2,
        L1,
        Ground_Ball,
        Processor,
        Low_Algae,
        High_Algae,
        Barge,
        Stow
    }
    public static Posistions targetpos=Posistions.Stow;
    LinearFilter Tempfilter1=LinearFilter.movingAverage(5);
    LinearFilter Tempfilter2=LinearFilter.movingAverage(5);
    private boolean OverrideTempLimit=false;
    public final SparkMax m_liftMotor1;
    public final SparkMax m_liftMotor2;
    public final RelativeEncoder m_liftEncoder;
    public final SparkClosedLoopController m_liftClosedLoopController;

    public final SparkMax m_pivotMotor1;
    public final AbsoluteEncoder m_pivotEncoder;
    public final SparkClosedLoopController m_pivotClosedLoopController;

    private final SparkBaseConfig liftMotor1Config;
    private final SparkBaseConfig liftMotor2Config;
    private final SparkBaseConfig pivotMotor1Config;

    private double liftClosedLoopReference = 0.0;
    private double pivotClosedLoopReference = 0.0;

    private int positionLevel = 0;

    private boolean liftAscending = true;


    public IntakePositionSubsystem() {
        
        // Configure Lift Motors
        m_liftMotor1 = new SparkMax(IntakePositionConstants.kLiftMotor1Id, MotorType.kBrushless);
        m_liftMotor2 = new SparkMax(IntakePositionConstants.kLiftMotor2Id, MotorType.kBrushless);
        m_liftEncoder = m_liftMotor2.getAlternateEncoder();
        m_liftClosedLoopController = m_liftMotor2.getClosedLoopController();

        SparkBaseConfig sharedLiftConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(60, 60);
        liftMotor1Config = new SparkMaxConfig().apply(sharedLiftConfig).follow(m_liftMotor2);
        liftMotor2Config = new SparkMaxConfig().apply(sharedLiftConfig).inverted(true);
        liftMotor2Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                                    .pid(IntakePositionConstants.kLiftPosP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftPosD)
                                    .iZone(IntakePositionConstants.kLiftIZone)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kLiftLoopTolerance);

        for(int i=0; i<=5; i++){
            m_liftMotor1.configure(liftMotor1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            m_liftMotor2.configure(liftMotor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        // Configure Pivot Motor
        m_pivotMotor1 = new SparkMax(IntakePositionConstants.kPivotMotor1Id, MotorType.kBrushless);
        m_pivotEncoder = m_pivotMotor1.getAbsoluteEncoder();
        m_pivotClosedLoopController = m_pivotMotor1.getClosedLoopController();

        pivotMotor1Config = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(45, 45);
        pivotMotor1Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                    .pid(IntakePositionConstants.kPivotP, IntakePositionConstants.kPivotI, IntakePositionConstants.kPivotD)
                                    .iZone(IntakePositionConstants.kPivotIZone)
                                    .positionWrappingInputRange(0, 1.0)
                                    .positionWrappingEnabled(true)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kPivotLoopTolerance);

        m_pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Go to Stow Position
        //setIntakePositionSetpoints(IntakePositionConstants.stowLift, IntakePositionConstants.stowPivot, 0);
    }
    
    public void OverrideTempLimit(){
        OverrideTempLimit=true;
    }

    public Command disableLiftCommand() {
        return new InstantCommand(()->{
            m_liftMotor1.disable();
            m_liftMotor2.disable();
        });
    }

    public Command disablePivotCommand() {
        return new InstantCommand(()->{
            m_pivotMotor1.disable();
        });
    }

    public void updateAll() {
        if((Tempfilter1.calculate(m_liftMotor1.getMotorTemperature())>Constants.IntakePositionConstants.kMaxLiftMotorTemp||Tempfilter2.calculate(m_liftMotor2.getMotorTemperature())>Constants.IntakePositionConstants.kMaxLiftMotorTemp)&&!OverrideTempLimit){
            System.out.println("WARNING LIFT MAX TEMP EXCEEDED");
            System.out.println("ELEVATOR WILL ATTEMPT TO STOW THEN DISABLE");
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new ParallelRaceGroup(new StowCommand(this), new WaitCommand(2)),
                disableLiftCommand()
            ));
            System.out.println("Good luck");
           
        }
        // Change lift P when changing between ascending/descending
        boolean liftAscendingNow = getLiftError() >= 0;
        if (liftAscendingNow != liftAscending) {
            liftAscending = liftAscendingNow;
            if (liftAscending) {
                liftMotor2Config.closedLoop.pid(IntakePositionConstants.kLiftPosP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftPosD);
            } else {
                liftMotor2Config.closedLoop.pid(IntakePositionConstants.kLiftNegP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftNegD);
            }
            m_liftMotor2.configure(liftMotor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public void zeroLift() {
        m_liftEncoder.setPosition(0);
    }

    public double getLiftPosition() {
        return m_liftEncoder.getPosition();
    }

    public double getLiftSetpoint() {
        return liftClosedLoopReference;
    }

    public void setLiftSetpoint(double setpoint, Integer forLevel) {
        if (forLevel != null) positionLevel = forLevel;
        liftClosedLoopReference = setpoint;
        m_liftClosedLoopController.setReference(liftClosedLoopReference, ControlType.kPosition, ClosedLoopSlot.kSlot0, IntakePositionConstants.kLiftAntigrav, ArbFFUnits.kPercentOut);
        //m_liftClosedLoopController.setReference(liftClosedLoopReference, ControlType.kPosition);
    }

    public void offsetLiftSetpoint(double delta) {
        setLiftSetpoint(liftClosedLoopReference + delta, null);
    }

    public void setLiftSpeed(double speed) {
        m_liftMotor2.set(speed);
    }

    public double getLiftError() {
        return getLiftSetpoint() - getLiftPosition();
    }

    public boolean liftAtTargetPos() {
        return Math.abs(getLiftError()) < IntakePositionConstants.kLiftTolerance;
    }

    public double getPivotPosition() {
        return m_pivotEncoder.getPosition();
    }

    public double getPivotSetpoint() {
        return pivotClosedLoopReference;
    }

    public void setPivotSetpoint(double setpoint, Integer forLevel) {
        if (forLevel != null) positionLevel = forLevel;
        pivotClosedLoopReference = setpoint;
        m_pivotClosedLoopController.setReference(pivotClosedLoopReference, ControlType.kPosition);
    }

    public void offsetPivotSetpoint(double delta) {
        setPivotSetpoint(pivotClosedLoopReference + delta, null);
    }

    public void setPivotSpeed(double speed) {
        m_pivotMotor1.set(speed);
    }

    public double getPivotError() {
        return getPivotSetpoint() - getPivotPosition();
    }

    public boolean pivotAtTargetPos() {
        return Math.abs(getPivotError()) < IntakePositionConstants.kPivotTolerance;
    }

    public void setIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint, Integer forLevel) {
        setLiftSetpoint(liftSetpoint, forLevel);
        setPivotSetpoint(pivotSetpoint, forLevel);
    }

    public void stopIntakePosition() {
        setLiftSpeed(0);
        setPivotSpeed(0);
    }

    public int getPositionLevel() {
        return positionLevel;
    }

    public boolean atTargetPos() {
        return liftAtTargetPos() && pivotAtTargetPos();
    }
    public double[] getCurrent(){
        double[] out={m_liftMotor1.getOutputCurrent(),m_liftMotor2.getOutputCurrent(),m_pivotMotor1.getOutputCurrent()};
        return out;
    }

    // private boolean stopAuto0=false;
    // public Command Auto0Lift=new FunctionalCommand(
    //     ()->setLiftSpeed(-0.2),
        
    //     () -> {
    //         if(lastPosisition<=getLiftPosition()){
    //             stopAuto0=true;
    //         }
    //         lastPosisition=getLiftPosition();
    //     },
    //     // stop lift and set 0
    //     interrupted -> {
    //     zeroLift();
    //     setLiftSpeed(0);
    //     stopAuto0=false;
    //     },

    //     //when stop auto0 stop it
    //     () -> stopAuto0,
    //     //require intake posistion
    //     this
        
    // );

}
