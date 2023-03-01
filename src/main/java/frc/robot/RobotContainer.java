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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import java.io.File;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;


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

    // The mechanismer's controller
    XboxController m_mechanismerController = new XboxController(OperatorConstants.kMechanismerControllerPort);

    // Power panel
    PowerDistribution m_PowerDistribution = new PowerDistribution(Constants.kPdpCanId, ModuleType.kRev);

    // Auto selection
    SendableChooser<String> m_chooser = new SendableChooser<>();
    String m_autoChoosed;

    Compressor m_compressor = new Compressor(PneumaticConstants.kPcmId, PneumaticsModuleType.REVPH);

    // are we field centric?
    boolean m_isFieldCentric = true;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // push PDP
        SmartDashboard.putData(m_PowerDistribution);

        // push gyro
        SmartDashboard.putData(m_robotDrive.m_gyro);

        // Configure the sendable chooser for auto picking
        for (String routine :
                getAutonomousRoutines()) {
            routine = removeFileExtension(routine);
            m_chooser.addOption(routine, routine);
            System.out.println("Added " + routine);
        }

        m_chooser.setDefaultOption("FullAuto", "FullAuto");
        SmartDashboard.putData(m_chooser);

        // Configure the button bindings
        configureButtonBindings();

        // Drive robot with the driver sticks
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getLeftY()), 0.01),
                                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getLeftX()), 0.01),
                                MathUtil.applyDeadband(driveStickCurve(-m_driverController.getRightX()), 0.01),
                                m_isFieldCentric),
                        m_robotDrive));

        // Extend and pivot arm with the mechanismer's sticks
        m_arm.setDefaultCommand(
                new RunCommand(
                        () -> {
                            m_arm.setExtendMotorSpeed(m_mechanismerController.getRightY());
                            m_arm.setPivotMotorSpeed(m_mechanismerController.getLeftY());
                        },
                        m_arm));

        m_compressor.enableAnalog(60, 120);
    }
//if(robot == true)[
//        Set.win=TRUE;
//]

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
        //region Driver
        // Hold the robot still when X is held
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        // Autobalance (Trevor touched this so double check this please)
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                .whileTrue(new BalanceCommand(m_robotDrive));

        // Toggle field-relative control
        new POVButton(m_driverController, 0)
                .toggleOnTrue(new RunCommand(() -> m_isFieldCentric = !m_isFieldCentric));
                //changed ".onTrue" to ".toggleOnTrue" (Trevor)
        //endregion

        //region Mechanismer
        // Close the claw
        new JoystickButton(m_mechanismerController, XboxController.Button.kLeftBumper.value)
                .whileTrue(new RunCommand(m_claw::closeClaw));

        // Open the claw
        new JoystickButton(m_mechanismerController, XboxController.Button.kRightBumper.value)
                .whileTrue(new RunCommand(m_claw::openClaw));

        // Move the claw in and the arm down
        // TODO: do this

        // Move the claw out and the arm out
        // TODO: do this
        //endregion
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_robotDrive.m_gyro.reset();
        m_autoChoosed = m_chooser.getSelected();
        // Create config for trajectory
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(m_autoChoosed, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        return m_robotDrive.autoBuilder.fullAuto(pathGroup);
    }

    private double driveStickCurve(double input) {
        return Math.copySign(3 * Math.pow(input, 3), input);
    }

    private Set<String> getAutonomousRoutines() {
        return Stream.of(Objects.requireNonNull(new File(Filesystem.getDeployDirectory() + "/pathplanner").listFiles())).filter(file -> !file.isDirectory()).map(File::getName).collect(Collectors.toSet());
    }

    private String removeFileExtension(String name) {
        if (name.indexOf(".") > 0) {
            return name.substring(0, name.lastIndexOf("."));
        } else {
            return name;
        }
    }
}