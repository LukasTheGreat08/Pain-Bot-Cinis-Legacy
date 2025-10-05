package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AidenBarriosOtherMechanism extends SubsystemBase {
    
    private final TalonFX m_barrios;
    
   
    public AidenBarriosOtherMechanism() {
        
        m_barrios  = new TalonFX(Constants.AidenBarriosOtherMechanism.MOTOR_ID, "rio");

    }

    public void runClimb(double power) {
        m_barrios.setControl(new DutyCycleOut(power));
        
    }

    public void reverseClimb(double power) {
        m_barrios.setControl(new DutyCycleOut(power));
    }

    public void stopClimb(){
        m_barrios.setControl(new NeutralOut());
    }

}
        
