package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperShooterSubsystem;

public class HopperShooterIntakeCommand extends Command {
    private final HopperShooterSubsystem m_subsystem;

    public HopperShooterIntakeCommand(HopperShooterSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.runIntake(Constants.HopperShooter.INTAKE_HOPPER_POWER, Constants.HopperShooter.INTAKE_POWER);
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.getDistance() < Constants.HopperShooter.CORAL_DETECT_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopAll();
    }
}