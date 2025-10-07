package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ABMSubsystem;
import frc.robot.subsystems.AidenBarriosMechanismSubsystem;
import frc.robot.subsystems.AidenBarriosOtherMechanism;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperPivotSubsystem;
import frc.robot.subsystems.HopperShooterSubsystem;


//ADIANBARRIOSINATOR - Stephen.R
public class ShenanigansCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final HopperShooterSubsystem m_hopperShooter;
    private final HopperPivotSubsystem m_hopperPivot;
    private final AidenBarriosMechanismSubsystem m_aidenBarrios;
    private final AidenBarriosOtherMechanism m_aidenBarriosOther;
    private final ABMSubsystem m_abm;
    private final Timer m_timer = new Timer();

    public ShenanigansCommand(
            CommandSwerveDrivetrain drivetrain,
            HopperShooterSubsystem hopperShooter,
            HopperPivotSubsystem hopperPivot,
            AidenBarriosMechanismSubsystem aidenBarrios,
            AidenBarriosOtherMechanism aidenBarriosOther,
            ABMSubsystem abm) {
        m_drivetrain = drivetrain;
        m_hopperShooter = hopperShooter;
        m_hopperPivot = hopperPivot;
        m_aidenBarrios = aidenBarrios;
        m_aidenBarriosOther = aidenBarriosOther;
        m_abm = abm;
        addRequirements(drivetrain, hopperShooter, hopperPivot, aidenBarrios, aidenBarriosOther, abm);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_drivetrain.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 10.0, true);
        m_hopperShooter.runIntake(1.0, 1.0);
        m_hopperPivot.setPower(1.0);
        m_aidenBarrios.fallDown();
        m_aidenBarriosOther.runClimb(1.0);
        m_abm.deploy();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stop();
        m_hopperShooter.stopAll();
        m_hopperPivot.stop();
        m_aidenBarrios.stop();
        m_aidenBarriosOther.stopClimb();
        m_abm.stop();
    }
}