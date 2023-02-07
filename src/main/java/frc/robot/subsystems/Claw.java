package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.pneumaticconstants;

public class Claw extends SubsystemBase{
    Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.REVPH, pneumaticconstants.solenoid);
    public void openClaw(){
        m_Solenoid.set(true);
    };
    public void closeClaw(){
        m_Solenoid.set(false);
    };
    public Claw() {}

}
    

