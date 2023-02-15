package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    CANSparkMax mClawOpenClose = new CANSparkMax(Constants.ClawConstants.kClawOpenCloseCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax mClawLeft = new CANSparkMax(Constants.ClawConstants.kClawLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax mClawRight = new CANSparkMax(Constants.ClawConstants.kClawRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void openClaw(){
       mClawOpenClose.set(-0.5);
    }
    public void closeClaw() {
        mClawOpenClose.set(0.5);
    }
    public Claw() {}

}
    

