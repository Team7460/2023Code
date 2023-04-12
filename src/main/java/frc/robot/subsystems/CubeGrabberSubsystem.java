package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CubeGrabberSubsystem extends SubsystemBase {
  private RobotContainer robotContainer;
  double setpoint = 0;

  public CANSparkMax m_pivotMotor =
      new CANSparkMax(
          Constants.CubeGrabberConstants.kPivotMotorCanId,
          CANSparkMaxLowLevel.MotorType.kBrushless);
  public CANSparkMax m_intakeMotor =
      new CANSparkMax(
          Constants.CubeGrabberConstants.kIntakeMotorCanId,
          CANSparkMaxLowLevel.MotorType.kBrushless);

  public CubeGrabberSubsystem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  public void setPivotMotorPositionSetpoint(double sp) {
    setpoint =
        MathUtil.clamp(
            sp,
            m_pivotMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse),
            m_pivotMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    m_pivotMotor.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getPivotMotorPosition() {
    return m_pivotMotor.getEncoder().getPosition();
  }

  public CommandBase bringIn() {
    return this.run(() -> setPivotMotorPositionSetpoint(1));
  }

  public CommandBase bringOut() {
    return this.run(() -> setPivotMotorPositionSetpoint(-10));
  }

  public CommandBase eatCube() {
    return this.run(() -> m_intakeMotor.set(1));
  }

  public CommandBase vomitCube() {
    return this.run(() -> m_intakeMotor.set(-1));
  }

}
