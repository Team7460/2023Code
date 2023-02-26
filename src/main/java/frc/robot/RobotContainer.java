// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final DriveSubsystem m_robotDrive = new DriveSubsystem();
  final ClawSubsystem m_claw = new ClawSubsystem();
  final ArmSubsystem m_arm = new ArmSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  // Power panel
  PowerDistribution m_PowerDistribution = new PowerDistribution(Constants.kPdpCanId, ModuleType.kRev);
  Compressor m_compressor = new Compressor(PneumaticConstants.kPcmId, PneumaticsModuleType.REVPH);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getLeftY()), 0.01),
                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getLeftX()), 0.01),
                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getRightX()), 0.01),
                true),
            m_robotDrive));

    m_arm.setDefaultCommand(new RunCommand(() -> m_arm.setExtendMotorSpeed(m_driverController.getLeftTriggerAxis()),m_arm));

    m_compressor.enableAnalog(60, 120);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileTrue(new RunCommand(m_claw::closeClaw, m_claw));
    // Hold the robot still when X is held
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new RunCommand(m_claw::openClaw, m_claw));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.m_gyro.reset();
    // Create config for trajectory
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return m_robotDrive.autoBuilder.fullAuto(pathGroup);
  }
  private double driveStickCurve(double input){
    return Math.copySign(3 * Math.pow(input, 3), input);
  }
}