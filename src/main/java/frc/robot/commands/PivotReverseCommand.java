package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperPivotSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class PivotReverseCommand extends Command {
    private final HopperPivotSubsystem m_pivotSubsystem;
    private final Timer m_timer = new Timer();
    private static final double TIMEOUT_SECONDS = 0.5;  // run for 1 second

    public PivotReverseCommand(HopperPivotSubsystem pivot) {
        m_pivotSubsystem = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        // Command the pivot motor to move downward (use your desired power)
        m_pivotSubsystem.setPower(0.375);
    }

    @Override
    public void execute() {
        // Optionally maintain power, but if not needed, nothing is required here.
    }

    @Override
    public boolean isFinished() {
        // End the command after 1 second has elapsed.
        return m_timer.get() >= TIMEOUT_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_pivotSubsystem.stop();
    }
}

