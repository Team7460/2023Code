package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;


public class BalanceCommand extends PIDCommand {

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        super(
                new PIDController(0.5, 0,0),
                driveSubsystem.getTilt,
                0,
                output -> driveSubsystem.drive(MathUtil.clamp(output, -0.15, 0.15), 0, 0, true, true),
                driveSubsystem
        );
        addRequirements(driveSubsystem);
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(2);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
