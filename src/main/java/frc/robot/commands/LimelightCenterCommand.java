package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Set;

public class LimelightCenterCommand implements Command {
  private final DriveSubsystem driveSubsystem;
  private final Set<Subsystem> subsystems;
  PIDController controller = new PIDController(0.5, 0, 0);

  public LimelightCenterCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.subsystems = Set.of(this.driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public Set<Subsystem> getRequirements() {
    return this.subsystems;
  }
}
