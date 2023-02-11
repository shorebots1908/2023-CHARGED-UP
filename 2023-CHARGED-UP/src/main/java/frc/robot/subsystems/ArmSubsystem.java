package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public class ArmSubsystem extends SubsystemBase{
    private MotorController armMotor1 = new PWMTalonFX(8);
    private MotorController armMotor2 = new PWMTalonFX(9);
    private double armSpeedLimit = 0.30;

    private MotorControllerGroup armMotors = new MotorControllerGroup(armMotor1, armMotor2);

    public ArmSubsystem() 
    {
        armMotor1.setInverted(true);
    }

    public void liftArm(double power) 
    {
        armMotors.set(armSpeedLimit * power);
    }

    public void lowerArm(double power)
    {
        armMotors.set(-armSpeedLimit * power);
    }
}
