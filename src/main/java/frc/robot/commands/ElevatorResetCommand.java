package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorResetCommand extends SequentialCommandGroup {
    public ElevatorResetCommand(ElevatorSubsystem elevator) {
        addCommands(
            new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.HEIGHT_L1), elevator),
            new WaitCommand(1),
            new InstantCommand(() -> elevator.stop())
        );
    }
}

