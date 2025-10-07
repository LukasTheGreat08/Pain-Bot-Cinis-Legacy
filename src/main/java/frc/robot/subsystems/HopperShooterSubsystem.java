package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperShooterSubsystem extends SubsystemBase {

    private final TalonFX m_hopperLeft;
    private final TalonFX m_hopperRight;
    private final TalonFX m_shooterRight;
    private final CANrange m_rangeSensor;

    private boolean isWindingDown = false;
    private double windDownPower = 0.0;

    public HopperShooterSubsystem() {
        m_hopperLeft  = new TalonFX(Constants.HopperShooter.HOPPER_LEFT_MOTOR_ID, "rio");
        m_hopperRight = new TalonFX(Constants.HopperShooter.HOPPER_RIGHT_MOTOR_ID, "rio");
        m_shooterRight = new TalonFX(Constants.HopperShooter.SHOOTER_RIGHT_MOTOR_ID, "rio");
        m_rangeSensor = new CANrange(Constants.HopperShooter.RANGE_SENSOR_ID, "rio");
    }

    @Override
    public void periodic() {
        if (isWindingDown) {
            if (windDownPower > 0) {
                windDownPower -= (Constants.HopperShooter.FIRE_POWER / (0.2 * 50.0)); // 2 seconds * 50Hz
                m_shooterRight.setControl(new DutyCycleOut(-windDownPower));
            } else {
                windDownPower = 0;
                isWindingDown = false;
                m_shooterRight.setControl(new NeutralOut());
            }
        }
    }

    public void runIntake(double hopper_power, double shooter_power) {
        isWindingDown = false;
        m_hopperLeft.setControl(new DutyCycleOut(-hopper_power)); // Reversed direction
        m_hopperRight.setControl(new DutyCycleOut(-hopper_power));
        m_shooterRight.setControl(new DutyCycleOut(-shooter_power));
    }

    public void runShooter(double power) {
        isWindingDown = false;
        m_shooterRight.setControl(new DutyCycleOut(-power));
    }

    public void runHopper(double power){
        isWindingDown = false;
        m_hopperLeft.setControl(new DutyCycleOut(power));
        m_hopperRight.setControl(new DutyCycleOut(-power));
    }

    public void runShooterL1(double power) {
        isWindingDown = false;
        m_shooterRight.setControl(new DutyCycleOut(-power));
    }

    public void runRewind(double power) {
        isWindingDown = false;
        m_hopperLeft.setControl(new DutyCycleOut(power));
        m_hopperRight.setControl(new DutyCycleOut(power));
        m_shooterRight.setControl(new DutyCycleOut(power));
    }

    public void stopAll() {
        isWindingDown = false;
        m_hopperLeft.setControl(new NeutralOut());
        m_hopperRight.setControl(new NeutralOut());
        m_shooterRight.setControl(new NeutralOut());
    }

    public void stopShooter() {
        if (!isWindingDown) {
            startWindDown();
        }
    }

    public void startWindDown() {
        isWindingDown = true;
        windDownPower = Constants.HopperShooter.FIRE_POWER;
    }

    public double getDistance() {
        return m_rangeSensor.getDistance().getValueAsDouble();
    }
}