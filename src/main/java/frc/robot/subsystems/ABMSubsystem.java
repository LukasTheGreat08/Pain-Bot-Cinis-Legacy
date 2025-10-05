package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ABMSubsystem extends SubsystemBase {

    private final TalonFX m_abmMotor;
    private boolean isDeployed = false;

    public ABMSubsystem() {
        m_abmMotor = new TalonFX(Constants.ABM.MOTOR_ID, "rio");
    }

    public void deploy() {
        m_abmMotor.setControl(new DutyCycleOut(Constants.ABM.DEPLOY_SPEED));
    }

    public void retract() {
        m_abmMotor.setControl(new DutyCycleOut(Constants.ABM.RETRACT_SPEED));
    }

    public void stop() {
        m_abmMotor.setControl(new NeutralOut());
    }

    public void toggle() {
        if (isDeployed) {
            retract();
        } else {
            deploy();
        }
        isDeployed = !isDeployed;
    }

    public boolean isDeployed() {
        return isDeployed;
    }
}