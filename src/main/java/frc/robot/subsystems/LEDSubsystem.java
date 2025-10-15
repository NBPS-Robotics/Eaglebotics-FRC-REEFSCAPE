package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final Spark m_Blinken;

    private static LEDSubsystem instance;

    public static LEDSubsystem getInstance(){//singleton because it makes it easier to use, can be switched if kelin wants
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    private LEDSubsystem(){
        m_Blinken= new Spark(5);
    }
    public void setStow(){
        m_Blinken.set(0.85);//dark blue
    }
    public void setBarge(){
        m_Blinken.set(0.67);//gold
    }
    public void setPlacePos(){
        m_Blinken.set(0.67);//gold
    }
    public void setOuttake(){
        m_Blinken.set(0.07);//flashes whatever we set color 1 as to fast
    }
    public void setIntake(){
        m_Blinken.set(0.27);//flashes whatever we set color 2 as to fast
    }



    public void setAlgaeMode(){
        if (m_Blinken.get() == 0.07 || m_Blinken.get() == 0.27) {
            m_Blinken.set(0.07); //Fast heartbeat for color 1, which should be Blue-Green
        } else {
            m_Blinken.set(0.79); //Green-Blue
        }
    }

    public void setCoralMode(){
        if (m_Blinken.get() == 0.07 || m_Blinken.get() == 0.27) {
            m_Blinken.set(0.27); //Fast heartbeat for color 2, which should be Red-Orange
        } else {
            m_Blinken.set(0.63); //Red-Orange
        }
    }

    //Called when the robot begins traveling to a set position, blinks whichever color the robot was already in
    public void setGoingToPos(){
        if (m_Blinken.get() == 0.79 || m_Blinken.get() == 0.07)
        {
            m_Blinken.set(0.07); //Fast heartbeat for color 1, which should be Blue-Green
        }
        else if (m_Blinken.get() == 0.63 || m_Blinken.get() == 0.27)
        {
            m_Blinken.set(0.27); //Fast heatbeat for color 2, which should be Red-Orange
        }
        else
        {
            m_Blinken.set(-0.87); //If the other two fail for some reason, Confetti Pattern!!!
        }
    }
    
}
