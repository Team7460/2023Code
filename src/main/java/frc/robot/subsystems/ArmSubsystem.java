package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax mArmPivotMotor =
      new CANSparkMax(
          Constants.ArmConstants.kArmMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax mArmExtendMotor =
      new CANSparkMax(
          Constants.ArmConstants.kArmExtendCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

  double setpoint = 0;

  public void setPivotMotorSpeed(double speed) {
    mArmPivotMotor.set(speed);
  }

  public void setExtendMotorSpeed(double speed) {
    mArmExtendMotor.set(speed);
  }

  public void setPivotMotorPositionSetpoint(double sp) {
    setpoint =
        MathUtil.clamp(
            sp,
            -mArmPivotMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse),
            mArmPivotMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    mArmPivotMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public double getPivotMotorPosition() {
    return mArmPivotMotor.getEncoder().getPosition();
  }

  public double getSetpoint() {
    return setpoint;
  }

  public ArmSubsystem(RobotContainer robotContainer) {
    mArmPivotMotor.getPIDController().setP(0.1);
  }
}
