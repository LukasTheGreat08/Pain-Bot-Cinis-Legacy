package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignRightCommand extends AutoAlignCommand {
  public AutoAlignRightCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    super(drivetrain, limelight, Constants.OPTIMAL_SCORING_POSE_RIGHT);
  }
}
