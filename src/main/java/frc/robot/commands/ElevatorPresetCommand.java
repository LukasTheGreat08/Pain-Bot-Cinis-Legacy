package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPresetCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private final double m_targetHeight;

    public ElevatorPresetCommand(ElevatorSubsystem elevator, double targetHeight) {
        m_elevator = elevator;
        m_targetHeight = targetHeight;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Optionally, log or store the target height.
    }

    @Override
    public void execute() {
        m_elevator.setTargetHeight(m_targetHeight);
    
    }

    @Override
    public boolean isFinished() {
        // This command holds indefinitely until interrupted.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
    }
}





