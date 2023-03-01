package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid mSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClawConstants.kForwardSolenoidId, Constants.ClawConstants.kReverseSolenoidId);

    public void closeClaw(){
       mSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void openClaw() {
        mSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public ClawSubsystem() {
        addChild("solenoid", mSolenoid);
    }
}
    

