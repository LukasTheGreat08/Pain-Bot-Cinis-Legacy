package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ABMSubsystem;

public class ToggleABMCommand extends Command {
    private final ABMSubsystem m_abmSubsystem;

    public ToggleABMCommand(ABMSubsystem subsystem) {
        m_abmSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_abmSubsystem.toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}