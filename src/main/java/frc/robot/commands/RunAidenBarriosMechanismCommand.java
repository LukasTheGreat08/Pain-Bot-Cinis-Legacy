package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AidenBarriosMechanismSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AidenBarriosMechanismSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RunAidenBarriosMechanismCommand extends SequentialCommandGroup{

        public RunAidenBarriosMechanismCommand(AidenBarriosMechanismSubsystem subsystem) {
            addCommands(
            // Step 1: Command the elevator to move to L1
            new InstantCommand(() -> subsystem.fallDown()),
            // Step 2: Wait 0.3 seconds to allow the elevator to settle at L1
            new WaitCommand(0.8),
            // Step 3: Fire the shooter, running for 0.5 seconds
            new InstantCommand(() -> subsystem.stop())
        );

        }
    }




