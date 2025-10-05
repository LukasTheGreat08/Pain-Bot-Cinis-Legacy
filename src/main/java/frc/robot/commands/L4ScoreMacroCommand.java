package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperShooterSubsystem;

public class L4ScoreMacroCommand extends SequentialCommandGroup {
    public L4ScoreMacroCommand(ElevatorSubsystem elevator, HopperShooterSubsystem shooterSubsystem) {
        addCommands(
            // Step 1: Command the elevator to move to L1
            new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.HEIGHT_L4), elevator),
            // Step 2: Wait 0.3 seconds to allow the elevator to settle at L1
            new WaitCommand(1),
            // Step 3: Fire the shooter, running for 0.5 seconds
            new HopperShooterFireCommand(shooterSubsystem).withTimeout(0.5),
            new ElevatorResetCommand((elevator))
        );
    }
}