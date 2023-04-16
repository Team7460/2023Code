package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Set;
import org.photonvision.PhotonCamera;

public class LimelightCenterCommand implements Command {
  private final DriveSubsystem driveSubsystem;
  private final Set<Subsystem> subsystems;
  PIDController controller = new PIDController(0.5, 0, 0);
  PhotonCamera camera;

  public LimelightCenterCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.subsystems = Set.of(this.driveSubsystem);
    this.camera = driveSubsystem.robotContainer.camera;
  }

  @Override
  public void initialize() {
    camera.setDriverMode(false);
  }

  @Override
  public void execute() {
    new TurnToAngleCommand(
        driveSubsystem, 180, driveSubsystem.robotContainer.m_driverController, false);
    // Get the latest result, feed setpoint into PID controller and then move
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    camera.setDriverMode(true);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return this.subsystems;
  }
}
