package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AidenBarriosMechanismSubsystem extends SubsystemBase {
    private final Servo m_servo;
  

    
    public AidenBarriosMechanismSubsystem() {
        m_servo = new Servo(Constants.AidenBarriosMechanism.SERVO_PWM_PORT);

    }

    public void fallDown() {
        m_servo.setPosition(0.4);
    }

    public void goUp() {
        m_servo.setPosition(0.6);
    }

    public void stop() {
        m_servo.setSpeed(0);
    }


}


