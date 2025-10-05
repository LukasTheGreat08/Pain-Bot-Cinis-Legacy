package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorEmergencyStopCommand extends Command {
    private final ElevatorSubsystem m_elevator;

    public ElevatorEmergencyStopCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
