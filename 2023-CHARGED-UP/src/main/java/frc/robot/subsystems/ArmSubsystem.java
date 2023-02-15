package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax armMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax armMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax wristMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    private double armSpeedLimit = 0.25;
    private double wristSpeedLimit = 0.25;
    private double[] armStates = {0.0, 0.0};

    private MotorControllerGroup armMotors = new MotorControllerGroup(armMotor1, armMotor2);

    public ArmSubsystem() 
    {
        armMotor1.setInverted(true);
        wristMotor1.setIdleMode(IdleMode.kBrake);
    }

    public void liftArm(double power) 
    {
        armMotors.set(armSpeedLimit * power);
    }

    public void lowerArm(double power)
    {
        armMotors.set(-armSpeedLimit * power);
    }

    public void wristMove(double power) 
    {
        wristMotor1.set(wristSpeedLimit * power);
    }

    public void setArmStates(double value, int index)
    {
        this.armStates[index] = value;
    }

    @Override
    public void periodic() {
        wristMove(this.armStates[1]);
        liftArm(this.armStates[0]);
    }
}
