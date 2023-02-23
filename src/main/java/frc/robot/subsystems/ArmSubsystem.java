package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    CANSparkMax mArmMotor = new CANSparkMax(Constants.ArmConstants.kArmMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

    public void setMotorSpeed(double speed){
        mArmMotor.set(speed);
    }
    public ArmSubsystem() {

    }
}

