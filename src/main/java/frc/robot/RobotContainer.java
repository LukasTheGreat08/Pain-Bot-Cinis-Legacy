// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.PivotReverseCommand;
import frc.robot.commands.ReverseAidenBarriosMechanismCommand;
import frc.robot.commands.ReverseAidenBarriosOtherMechanismCommand;
import frc.robot.commands.RunAidenBarriosMechanismCommand;
import frc.robot.commands.RunAidenBarriosOtherMechanismCommand;
import frc.robot.commands.ToggleABMCommand;
import frc.robot.commands.ElevatorPresetCommand;
import frc.robot.commands.ElevatorResetCommand;
import frc.robot.commands.HopperIntake;
import frc.robot.commands.HopperRewindCommand;
import frc.robot.commands.HopperShooterFireCommand;
import frc.robot.commands.HopperShooterIntakeCommand;
import frc.robot.commands.HopperShooterRewindCommand;
import frc.robot.commands.L1FireCommand;
import frc.robot.commands.L1ScoreMacroCommand;
import frc.robot.commands.L4ScoreMacroCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.AutoAlignLeftCommand;
import frc.robot.commands.AutoAlignRightCommand;
import frc.robot.commands.ElevatorEmergencyStopCommand;
import frc.robot.subsystems.AidenBarriosMechanismSubsystem;
import frc.robot.subsystems.AidenBarriosOtherMechanism;
import frc.robot.subsystems.AidenBarriosOtherMechanism;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperPivotSubsystem;
import frc.robot.subsystems.HopperShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ABMSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;


public class RobotContainer {

    private final Joystick m_buttonBox = new Joystick(Constants.OI.BUTTON_BOX_PORT);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final HopperPivotSubsystem m_hopperPivotSubsystem = new HopperPivotSubsystem();
    private final HopperShooterSubsystem m_hopperShooterSubsystem = new HopperShooterSubsystem();
    private final AidenBarriosMechanismSubsystem m_mechanism = new AidenBarriosMechanismSubsystem();
    private final AidenBarriosOtherMechanism m_otherMechanism = new AidenBarriosOtherMechanism();
    private final LimelightSubsystem m_rizz = new LimelightSubsystem();
    private final ABMSubsystem m_abmSubsystem = new ABMSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Fire", new HopperShooterFireCommand(m_hopperShooterSubsystem));
        NamedCommands.registerCommand("Elevator Reset",  new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_RESET));
        NamedCommands.registerCommand("Elevator L4",  new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_L4));
        NamedCommands.registerCommand("Intake",  new HopperShooterIntakeCommand(m_hopperShooterSubsystem));
        NamedCommands.registerCommand("L4 Score",  new L4ScoreMacroCommand(m_elevatorSubsystem, m_hopperShooterSubsystem));
        NamedCommands.registerCommand("L1 Score",  new L1ScoreMacroCommand(m_elevatorSubsystem, m_hopperShooterSubsystem));
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);


        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );




        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Hopper Shooter Commands
        joystick.rightTrigger().whileTrue(new HopperShooterIntakeCommand(m_hopperShooterSubsystem));
        joystick.leftTrigger().whileTrue(new HopperShooterRewindCommand(m_hopperShooterSubsystem));
        joystick.a().whileTrue(new HopperShooterFireCommand(m_hopperShooterSubsystem));


        drivetrain.registerTelemetry(logger::telemeterize);


        // HopperShooter commands
        new JoystickButton(m_buttonBox, 1)
            .whileTrue(new PivotCommand(m_hopperPivotSubsystem));
        new JoystickButton(m_buttonBox, 2)
            .whileTrue(new PivotReverseCommand(m_hopperPivotSubsystem));
        new JoystickButton(m_buttonBox, 3)
            .whileTrue(new HopperRewindCommand(m_hopperShooterSubsystem));
        new JoystickButton(m_buttonBox, 4)
            .whileTrue(new L1FireCommand(m_hopperShooterSubsystem));

        // Elevator presets
        new JoystickButton(m_buttonBox, 10)
            .onTrue(new ElevatorResetCommand(m_elevatorSubsystem));
        new JoystickButton(m_buttonBox, 6)
            .onTrue(new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_L1));
        new JoystickButton(m_buttonBox, 7)
            .onTrue(new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_L2));
        new JoystickButton(m_buttonBox, 8)
            .onTrue(new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_L3));
        new JoystickButton(m_buttonBox, 9)
            .onTrue(new ElevatorPresetCommand(m_elevatorSubsystem, Constants.Elevator.HEIGHT_L4));

        // Pivot commands
        new JoystickButton(m_buttonBox, 11)
            .whileTrue(new RunAidenBarriosOtherMechanismCommand(m_otherMechanism)); //into chassis
        new JoystickButton(m_buttonBox, 12)
            .whileTrue(new ReverseAidenBarriosOtherMechanismCommand(m_otherMechanism )); //out of chassis

        //new JoystickButton(m_buttonBox, 5)
            //.onTrue(new AutoAlignLeftCommand(drivetrain, m_rizz));
        //new JoystickButton(m_buttonBox, 4)
            //.onTrue(new AutoAlignRightCommand(drivetrain,m_rizz));

            //FORBIDDEN TECH DO NOT TOUCH





        //aiden barrios's mechanisms
        joystick.x().whileTrue(new RunAidenBarriosMechanismCommand(m_mechanism));
        joystick.y().whileTrue(new ReverseAidenBarriosMechanismCommand(m_mechanism));

        // New ABM mapping
        joystick.b().onTrue(new ToggleABMCommand(m_abmSubsystem));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}