package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Set;

public class LimelightCenterCommand implements Command {
  private final DriveSubsystem driveSubsystem;
  private final Set<Subsystem> subsystems;
  PIDController controller = new PIDController(0.5, 0, 0);

  NetworkTable m_table;

  public LimelightCenterCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.subsystems = Set.of(this.driveSubsystem);
  }

  @Override
  public void initialize() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_table.getEntry("camMode").setNumber(0); // Set into vision processing mode
    m_table.getEntry("ledMode").setNumber(0); // Turn leds on
    controller.setTolerance(3);
  }

  @Override
  public void execute() {
    Number tx = m_table.getEntry("tx").getNumber(0);
    double calculated = controller.calculate((Double) tx, 0);
    driveSubsystem.drive(0, calculated, 0, false, false);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_table.getEntry("camMode").setNumber(1); // Set into driver mode
    m_table.getEntry("ledMode").setNumber(1); // Turn leds off
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return this.subsystems;
  }
}
