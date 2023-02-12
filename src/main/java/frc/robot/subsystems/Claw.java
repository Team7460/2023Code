package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PneumaticConstants;

public class Claw extends SubsystemBase {
    DoubleSolenoid mDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.kClawForwardSolenoidId, PneumaticConstants.kClawBackwardSolenoidId);
    public void openClaw(){
       mDoubleSolenoid.set(Value.kForward);
    }
    public void closeClaw(){
        mDoubleSolenoid.set(Value.kReverse);
    }
    public Claw() {}

}
    

