package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public final class Constants {
   
    public static final Pose2d OPTIMAL_SCORING_POSE_LEFT = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(90));
    public static final Pose2d OPTIMAL_SCORING_POSE_RIGHT = new Pose2d(1.5, -2.0, Rotation2d.fromDegrees(90));

    public static final class OI {
        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int BUTTON_BOX_PORT = 1;
    }
    
    public static final class HopperPivot {
        //775pro on top of hopper
        public static final int MOTOR_ID = 44; // fixed
    }

    public static final class AidenBarriosMechanism {
            public static final int SERVO_PWM_PORT = 0;//maybe fix
    }
    
    
    
    public static final class AidenBarriosOtherMechanism {
       //climb motor
        public static final int MOTOR_ID = 35;//fixed
        
        
        public static final double FIRE_POWER = .75;
        
        
        public static final double REVERSE_POWER = -.75;
    }

    public static final class Elevator {
    
        public static final int LEFT_MOTOR_ID = 1;
        public static final int RIGHT_MOTOR_ID = 2;

        
        public static final double HEIGHT_PER_ROTATION = 0.05;  // Example: 5 cm per rotation

        //PSA: these are in meters
        public static final double HEIGHT_RESET = 0.0;   
        public static final double HEIGHT_L1 = 0.325;       
        public static final double HEIGHT_L2 = 0.69;       
        public static final double HEIGHT_L3 = 1.28;       
        public static final double HEIGHT_L4 = 2.236;       

        
    }




    public static final class Swerve {
    // Module IDs (all fixed)
        public static final int FRONT_LEFT_DRIVE_ID    = 7;
        public static final int FRONT_LEFT_TURN_ID     = 6;
        public static final int FRONT_LEFT_CANCODER_ID = 8;

        public static final int FRONT_RIGHT_DRIVE_ID    = 9;
        public static final int FRONT_RIGHT_TURN_ID     = 10;
        public static final int FRONT_RIGHT_CANCODER_ID = 11;

        public static final int BACK_LEFT_DRIVE_ID    = 4;
        public static final int BACK_LEFT_TURN_ID     = 3;
        public static final int BACK_LEFT_CANCODER_ID = 5;

        public static final int BACK_RIGHT_DRIVE_ID    = 12;
        public static final int BACK_RIGHT_TURN_ID     = 13;
        public static final int BACK_RIGHT_CANCODER_ID = 14;

    
        public static final double FRONT_LEFT_OFFSET_DEG  = 113.29092;
        public static final double FRONT_RIGHT_OFFSET_DEG = -108.28116;
        public static final double BACK_LEFT_OFFSET_DEG   = 72.94932;
        public static final double BACK_RIGHT_OFFSET_DEG  = 112.5;

    
        public static final int PIGEON_ID = 58; //good!

        // meters
        public static final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches 
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TRACK_WIDTH = 0.6096; //24 inches
        public static final double WHEEL_BASE  = 0.6096; //24 inches (maybe fix)

        // Gear ratios
        public static final double DRIVE_GEAR_RATIO = 5.9;  // Motor rotations per wheel rotation
        public static final double TURN_GEAR_RATIO  = 18.75;  // Motor rotations per module rotation

        //turn tuning
        public static final double TURN_kP = 100;
        public static final double TURN_kI = 0.0;
        public static final double TURN_kD = 0.0;

        //drive tuning
        public static final double DRIVE_kP = 0.2;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        // Max speeds
        public static final double MAX_SPEED = 6.0;              // m/s
        public static final double MAX_ANGULAR_SPEED = 3.0 * Math.PI; // rad/s

        // Swerve Drive Kinematics (using module positions relative to the robot center)
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0,  TRACK_WIDTH / 2.0),   // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),   // Front Right
            new Translation2d(-WHEEL_BASE / 2.0,  TRACK_WIDTH / 2.0),  // Back Left
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)   // Back Right
        );
    }
        
    public static final class HopperShooter {
        //fix all these!!!! **************************************************
        public static final int HOPPER_LEFT_MOTOR_ID = 21;
        public static final int HOPPER_RIGHT_MOTOR_ID = 20;
        public static final int SHOOTER_LEFT_MOTOR_ID = 28;
        public static final int SHOOTER_RIGHT_MOTOR_ID = 25;
        
        
        public static final int RANGE_SENSOR_ID = 17;

        public static final double INTAKE_POWER = 0.05;
        public static final double INTAKE_HOPPER_POWER = 0.85;
        public static final double FIRE_POWER   = 0.2;
        public static final double REWIND_POWER = 0.25;
        public static final double L1FIRE = 0.2575;

        
        public static final double CORAL_DETECT_THRESHOLD = 0.1; //m
    }
}


    

