package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid mLeftSide = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClawConstants.kLeftForwardSolenoidId, Constants.ClawConstants.kLeftReverseSolenoidId);
    DoubleSolenoid mRightSide = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ClawConstants.kRightForwardSolenoidId, Constants.ClawConstants.kRightReverseSolenoidId);

    public void closeClaw(){
       mLeftSide.set(DoubleSolenoid.Value.kForward);
       mRightSide.set(DoubleSolenoid.Value.kForward);
    }
    public void openClaw() {
        mLeftSide.set(DoubleSolenoid.Value.kReverse);
        mRightSide.set(DoubleSolenoid.Value.kReverse);
    }

    public ClawSubsystem() {
        addChild("leftSide", mLeftSide);
        addChild("rightSide", mRightSide);
    }
}
    

