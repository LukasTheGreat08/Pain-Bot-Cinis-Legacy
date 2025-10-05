package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperPivotSubsystem extends SubsystemBase {
    private final SparkMax m_pivotMotor;

    public HopperPivotSubsystem() {
        
        m_pivotMotor = new SparkMax(Constants.HopperPivot.MOTOR_ID, MotorType.kBrushed);

       
        SparkMaxConfig config = new SparkMaxConfig();

        
        config.inverted(true);

    
        m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPower(double power) {
        m_pivotMotor.set(power);
    }

   
    public void stop() {
        setPower(0.0);
    }
}