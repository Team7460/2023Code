package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;

  private int state;
  private int debounceCount;
  private final double robotSpeedSlow;
  private final double robotSpeedFast;
  private final double onChargeStationDegree;
  private final double levelDegree;
  private final double debounceTime;

  private int tickDivider;

  private long autoBalanceTicks;
  public BalanceCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(this.driveSubsystem);

    state = 0;
    debounceCount = 0;

    // Speed the robot drives while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.12;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.0695;

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 12.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 6.0;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noise, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.1;

    tickDivider = 80;
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    double toDrive = -calculateMotorSpeeds();
    if (toDrive == 0){
      driveSubsystem.setX();
    } else {
      driveSubsystem.drive(toDrive, 0, 0, false, false);
    }
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
    return state == 3;
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
    driveSubsystem.setX();
  }

  public double calculateMotorSpeeds() {
    switch (state) {
        // drive forwards to approach station, exit when tilt is detected
      case 0:
        if (-driveSubsystem.m_gyro.getRoll() > onChargeStationDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          return robotSpeedSlow;
        }
        return robotSpeedFast;
        // driving up charge station, drive slower, stopping when level
      case 1:
        autoBalanceTicks++;
        if (-driveSubsystem.m_gyro.getRoll() < levelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          debounceCount = 0;
          return 0;
        }
        if (autoBalanceTicks % 100 > tickDivider) {
          if(autoBalanceTicks % 100 == tickDivider) {
            if(tickDivider > 10) {
              tickDivider -= 5;
            }
          }
          driveSubsystem.setX();
          return 0;
        }

        return robotSpeedSlow;
        // on charge station, stop motors and wait for end of auto
      case 2:
        if (Math.abs(-driveSubsystem.m_gyro.getRoll()) <= levelDegree / 2) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 4;
          debounceCount = 0;
          return 0;
        }
        if (-driveSubsystem.m_gyro.getRoll() >= levelDegree) {
          return 0.1;
        } else if (-driveSubsystem.m_gyro.getRoll() <= -levelDegree) {
          return -0.1;
        }
      case 3:
        return 0;
    }
    return 0;
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }
}
