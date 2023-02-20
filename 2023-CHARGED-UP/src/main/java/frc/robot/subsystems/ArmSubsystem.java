package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 

public class ArmSubsystem extends SubsystemBase{
    //motor definitions
    
    private CANSparkMax armMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax armMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax wristMotor1 = new CANSparkMax(15, MotorType.kBrushless);

    //state management parameters
    private double armSpeedLimit = 0.25;
    private double wristSpeedLimit = 0.25;
    private double[] armStates = {0.0, 0.0};
    public enum ArmJoint {
        Shoulder(0),
        Wrist(1);

        public final int value;

        ArmJoint(int value) {
            this.value = value;
        }
    }

    //encoders
    private RelativeEncoder armEncoder;
    private RelativeEncoder wristEncoder;


    private double minSpeed = 0.05;
    private double maxSpeed = 0.4;

    //hold position settings
    //TODO: set proper values based on encoder readouts.
    private double deviation = 10;
    private double HighPosition;
    private double MidPosition;
    private double LowPosition;
    private double StowPosition;
    private double wristOffset;

    private double motorRatios = 27.0 / 400.0;

    private MotorControllerGroup armMotors = new MotorControllerGroup(armMotor1, armMotor2);

    public ArmSubsystem() 
    {
        armMotor1.setInverted(true);
        wristMotor1.setIdleMode(IdleMode.kBrake);
        armEncoder = armMotor1.getEncoder();
        wristEncoder = wristMotor1.getEncoder();
        SmartDashboard.getNumber("High Position", HighPosition);
        SmartDashboard.getNumber("Middle Position", MidPosition);
        SmartDashboard.getNumber("Lower Position", LowPosition);
        SmartDashboard.getNumber("Stowed Position", StowPosition);

        SmartDashboard.putNumber("High Position", HighPosition);
        SmartDashboard.putNumber("Middle Position", MidPosition);
        SmartDashboard.putNumber("Lower Position", LowPosition);
        SmartDashboard.putNumber("Stowed Position", StowPosition);

        SmartDashboard.getNumber("Wrist Offset", wristOffset);
        SmartDashboard.putNumber("Wrist Offset", wristOffset);

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
private double seekSpeed(double desiredPosition) {
    double currentPosition = armEncoder.getPosition();
    double outputSpeed = (desiredPosition - currentPosition) * 0.1;
        if(outputSpeed < -maxSpeed) {
            return -maxSpeed;
        }
        else if(outputSpeed > maxSpeed) {
            return maxSpeed;
        }
        else if(outputSpeed > -minSpeed && outputSpeed < 0) {
            return -minSpeed;
        }
        else if(outputSpeed < minSpeed && outputSpeed > 0) {
            return minSpeed;
        }
        else {
            return outputSpeed;
        }
    }

    public void armHold(double desiredPosition) {

        if(Math.abs(armEncoder.getPosition() - desiredPosition) > deviation)
        {
            this.armStates[ArmJoint.Shoulder.value] = seekSpeed(desiredPosition);
        }
        else
        {
            this.armStates[ArmJoint.Shoulder.value] = 0;
        }
    }

    public void wristHold(double desiredPosition) {

        if(Math.abs(wristEncoder.getPosition() - desiredPosition) > deviation)
        {
            this.armStates[ArmJoint.Wrist.value] = seekSpeed(desiredPosition);
        } 
        else 
        {
            this.armStates[ArmJoint.Wrist.value] = 0;
        }
    }

    public double offsetWristPosition() {
        return (armEncoder.getPosition() - LowPosition) * motorRatios - wristOffset;
    }

    @Override
    public void periodic() {
        wristMove(this.armStates[ArmJoint.Wrist.value]);
        liftArm(this.armStates[ArmJoint.Shoulder.value]);
        SmartDashboard.putNumber("armMotor1", armEncoder.getPosition());
        SmartDashboard.putNumber("wristMotor1", wristEncoder.getPosition());
        SmartDashboard.getNumber("High Position", HighPosition);
        SmartDashboard.getNumber("Middle Position", MidPosition);
        SmartDashboard.getNumber("Lower Position", LowPosition);
        SmartDashboard.getNumber("Stowed Position", StowPosition);
        SmartDashboard.getNumber("Wrist Offset", wristOffset);
    }
}