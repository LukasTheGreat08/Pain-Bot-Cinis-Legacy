package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final Pose2d targetPose;
  
  // PID controllers for the X, Y and rotational corrections.
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;
  
  // Tolerances (adjust as needed)
  private static final double POSITION_TOLERANCE_METERS = 0.1;
  private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.0);

  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.targetPose = targetPose;
    xController = new PIDController(0.5, 0.0, 0.1);
    yController = new PIDController(0.5, 0.0, 0.1);
    thetaController = new PIDController(0.05, 0.0, 0.01);
    
    // Set tolerances for each controller.
    xController.setTolerance(POSITION_TOLERANCE_METERS);
    yController.setTolerance(POSITION_TOLERANCE_METERS);
    thetaController.setTolerance(ANGLE_TOLERANCE_RADIANS);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
    
    // Set the target for each PID controller based on the optimal scoring pose.
    xController.setSetpoint(targetPose.getTranslation().getX());
    yController.setSetpoint(targetPose.getTranslation().getY());
    thetaController.setSetpoint(targetPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    // Get current pose from vision; ensure your LimelightSubsystem returns a current Pose2d.
    Pose2d currentPose = limelight.getPose();
    
    // Calculate corrections for translation and rotation.
    double correctionX = xController.calculate(currentPose.getTranslation().getX());
    double correctionY = yController.calculate(currentPose.getTranslation().getY());
    double correctionTheta = thetaController.calculate(currentPose.getRotation().getRadians());
    
    // Command the drivetrain to drive with the calculated corrections.
    // This example uses a helper method "drive(Translation2d, double, boolean)" for field-centric control.
    drivetrain.drive(new Translation2d(correctionX, correctionY), correctionTheta, true);
  }

  @Override
  public boolean isFinished() {
    // The command finishes when all PID controllers are at their setpoints.
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
