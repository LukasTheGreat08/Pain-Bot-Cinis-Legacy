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
    
    
    private final TalonFX m_shooterLeft;
    private final TalonFX m_shooterRight;
    
    // CTRE CANrange sensor
    private final CANrange m_rangeSensor;
    
    public HopperShooterSubsystem() {
        
        m_hopperLeft  = new TalonFX(Constants.HopperShooter.HOPPER_LEFT_MOTOR_ID, "rio");
        m_hopperRight = new TalonFX(Constants.HopperShooter.HOPPER_RIGHT_MOTOR_ID, "rio");
        
        
        m_shooterLeft  = new TalonFX(Constants.HopperShooter.SHOOTER_LEFT_MOTOR_ID, "rio");
        m_shooterRight = new TalonFX(Constants.HopperShooter.SHOOTER_RIGHT_MOTOR_ID, "rio");
        
        
        m_rangeSensor = new CANrange(Constants.HopperShooter.RANGE_SENSOR_ID, "rio");
    }
    
    
    public void runIntake(double hopper_power, double shooter_power) {
        m_hopperLeft.setControl(new DutyCycleOut(-hopper_power));
        m_hopperRight.setControl(new DutyCycleOut(-hopper_power));
        m_shooterLeft.setControl(new DutyCycleOut(shooter_power));
        m_shooterRight.setControl(new DutyCycleOut(-shooter_power));
    }
    
    
    public void runShooter(double power) {
        m_shooterLeft.setControl(new DutyCycleOut(power));
        m_shooterRight.setControl(new DutyCycleOut(-power));
    }

    public void runHopper(double power){
        m_hopperLeft.setControl(new DutyCycleOut(-power));
        m_hopperRight.setControl(new DutyCycleOut(-power));
    }

    public void runShooterL1(double power) {
        m_shooterLeft.setControl(new DutyCycleOut(power-0.2));
        m_shooterRight.setControl(new DutyCycleOut(-power));
    }
    
    
    public void runRewind(double power) {
        m_hopperLeft.setControl(new DutyCycleOut(power));
        m_hopperRight.setControl(new DutyCycleOut(power));
        m_shooterLeft.setControl(new DutyCycleOut(-power));
        m_shooterRight.setControl(new DutyCycleOut(power));
    }
    
    
    public void stopAll() {
        m_hopperLeft.setControl(new NeutralOut());
        m_hopperRight.setControl(new NeutralOut());
        m_shooterLeft.setControl(new NeutralOut());
        m_shooterRight.setControl(new NeutralOut());
    }
    
    
    public void stopShooter() {
        m_shooterLeft.setControl(new NeutralOut());
        m_shooterRight.setControl(new NeutralOut());
    }
    
    
    public double getDistance() {
        return m_rangeSensor.getDistance().getValueAsDouble();
    }
}


