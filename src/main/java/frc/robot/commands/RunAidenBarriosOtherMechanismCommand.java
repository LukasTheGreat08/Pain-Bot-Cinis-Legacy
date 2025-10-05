package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AidenBarriosOtherMechanism;

public class RunAidenBarriosOtherMechanismCommand extends Command {
    private final AidenBarriosOtherMechanism m_otherSubsystem;
    
    public RunAidenBarriosOtherMechanismCommand(AidenBarriosOtherMechanism subsystem) {
        m_otherSubsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void execute() {
        m_otherSubsystem.runClimb(Constants.AidenBarriosOtherMechanism.FIRE_POWER);
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until the button is released.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_otherSubsystem.stopClimb();
    }
}

