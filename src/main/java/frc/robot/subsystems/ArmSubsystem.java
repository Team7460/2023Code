package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    CANSparkMax mArmPivotMotor = new CANSparkMax(Constants.ArmConstants.kArmMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setPivotMotorSpeed(double speed){
        mArmPivotMotor.set(speed);
    }
    public ArmSubsystem() {

    }
}

