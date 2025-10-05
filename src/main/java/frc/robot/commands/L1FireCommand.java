package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperShooterSubsystem;

public class L1FireCommand extends Command {
    private final HopperShooterSubsystem m_subsystem;

    public L1FireCommand(HopperShooterSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
        
    }

    @Override
    public void execute() {
        m_subsystem.runShooterL1(Constants.HopperShooter.L1FIRE);
        
    }

    @Override
    public boolean isFinished() {
        //runs until button no press no more
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopShooter();
    }
}