package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperShooterSubsystem;

public class HopperRewindCommand extends Command {
    private final HopperShooterSubsystem m_subsystem;

    public HopperRewindCommand(HopperShooterSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.runRewind(Constants.HopperShooter.REWIND_POWER);
    }

    @Override
    public boolean isFinished() {
        // Typically runs until button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopAll();
    }
}
