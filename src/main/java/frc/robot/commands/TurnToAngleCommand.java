package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private PIDController controller = new PIDController(0.4, 0, 0);
  private final double angle;

  public TurnToAngleCommand(DriveSubsystem driveSubsystem, double angle) {
    this.angle = angle;
    this.driveSubsystem = driveSubsystem;
    addRequirements(this.driveSubsystem);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(Units.degreesToRadians(4));
    SmartDashboard.putData("TurnPID", controller);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    SmartDashboard.putNumber("Turn Error", controller.getPositionError());
    double output =
        controller.calculate(Units.degreesToRadians(driveSubsystem.m_gyro.getYaw()), this.angle);
    output = MathUtil.clamp(output, -1, 1);
    driveSubsystem.drive(0, 0, -output, false, false);
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is it is called when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
   * motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false, false);
  }
}
