package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;
    private final double TOLERANCE = 0.02; // 2 cm tolerance

    // Record the encoder reading when the elevator is at its bottom (reset) position.
    private double baselineRotations;

    public ElevatorSubsystem() {
        m_leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID, "rio");
        m_rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID, "rio");

        // Configure the motors for Motion Magic.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 4.8;
        config.Slot0.kD = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 110;   //110 prty fast //300 very fast
        config.MotionMagic.MotionMagicAcceleration = 260;    //prty fast 260 //supa fast 420
        config.MotionMagic.MotionMagicJerk = 2000;
        m_leftMotor.getConfigurator().apply(config);
        m_rightMotor.getConfigurator().apply(config);

        // Capture the baseline (reset) encoder reading.
        // Assume that when the robot starts, the elevator is at the bottom.
        baselineRotations = m_leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Commands the elevator to move to the specified target height (in meters)
     * using Motion Magic. This calculation uses a baseline offset.
     * Assumes that as the elevator rises, the encoder value decreases.
     *
     * @param heightMeters the desired elevator height in meters
     */
    public void setTargetHeight(double heightMeters) {
        double targetRotations = baselineRotations - (heightMeters / Constants.Elevator.HEIGHT_PER_ROTATION);
        MotionMagicVoltage request = new MotionMagicVoltage(0).withPosition(targetRotations);
        m_leftMotor.setControl(request);
        m_rightMotor.setControl(request);
    }

    /**
     * Holds the elevator in a “reset” state. In this mode, we assume the motors
     * are configured (or allowed) to be in coast mode and we apply a small open-loop
     * output to fight gravity so the elevator doesn’t crash.
     */
    public void setResetHold() {
        // Apply a small constant duty cycle (e.g., 0.1) upward.
        m_leftMotor.setControl(new DutyCycleOut(0.1));
        m_rightMotor.setControl(new DutyCycleOut(0.1));
    }

    /**
     * Stops the elevator by commanding neutral output.
     */
    public void stop() {
        m_leftMotor.setControl(new NeutralOut());
        m_rightMotor.setControl(new NeutralOut());
    }

    /**
     * Returns the current elevator height in meters based on the left motor encoder.
     */
    public double getHeight() {
        double rotations = m_leftMotor.getPosition().getValueAsDouble();
        // Compute height as the difference from the baseline.
        return (baselineRotations - rotations) * Constants.Elevator.HEIGHT_PER_ROTATION;
    }

    /**
     * Checks whether the elevator is within tolerance of a target height.
     *
     * @param targetHeight the desired height in meters
     * @return true if within tolerance, false otherwise
     */
    public boolean isAtTarget(double targetHeight) {
        return Math.abs(getHeight() - targetHeight) <= TOLERANCE;
    }
}

