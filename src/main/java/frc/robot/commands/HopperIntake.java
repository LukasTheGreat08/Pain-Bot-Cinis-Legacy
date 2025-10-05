package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperShooterSubsystem;

public class HopperIntake extends Command {
    private final HopperShooterSubsystem m_subsystem;

    public HopperIntake(HopperShooterSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        
        m_subsystem.runHopper(Constants.HopperShooter.INTAKE_HOPPER_POWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopAll();
    }
}
