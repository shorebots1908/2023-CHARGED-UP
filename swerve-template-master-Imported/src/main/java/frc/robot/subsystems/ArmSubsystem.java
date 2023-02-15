package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public class ArmSubsystem extends SubsystemBase {
    //TODO : GET ENCODER VALUES
    private final PWMMotorController m_ShoulderMotor = new PWMTalonFX(14);
    private final PWMMotorController m_WristMotor = new PWMTalonFX(15);
    public ArmSubsystem() {

    }
public void storageMode() {
    if(
        //check wrist encoder
    )
    {
        m_WristMotor.set(0.2);
    }
    else
    {
        m_WristMotor.set(0);
    }

    if(
        //check shoulder encoder
    )
    {
        m_ShoulderMotor.set(-0.2);
    }
    else 
    {
        m_ShoulderMotor.set(0);
    }
}

}
