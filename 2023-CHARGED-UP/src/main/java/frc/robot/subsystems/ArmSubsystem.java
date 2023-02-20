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

    //state management parameterd
    private double armSpeedLimit = 0.25;
    private double wristSpeedLimit = 0.25;
    private double[] armStates = {0.0, 0.0};

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
    private double wristHoldPostion = 0.0;

    private MotorControllerGroup armMotors = new MotorControllerGroup(armMotor1, armMotor2);

    public ArmSubsystem(){
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

    }

    public void liftArm(double power) {
        armMotors.set(armSpeedLimit * power);
    }

    public void lowerArm(double power){
        armMotors.set(-armSpeedLimit * power);
    }

    public void wristMove(double power){
        wristMotor1.set(wristSpeedLimit * power);
    }

    public void wristUp(){ //Triggered by 'A' Button
        wristMove(0.1);
    }
    
    public void wristDown(){ //Triggered by 'B' Button
        wristMove(-0.1);
    }

    public void setWristHoldPosition(){ //Triggered by 'Y' Button Once
        wristHoldPostion = wristEncoder.getPosition();
    }

    public void wristHold(){ //Triggered by 'Y' Button while Held
        if(Math.abs(armEncoder.getPosition() - wristHoldPostion) > deviation){
            this.armStates[1] = seekSpeed(wristHoldPostion, wristEncoder.getPosition());
        } else {
            this.armStates[1] = 0;
        }
    }

    public void setArmStates(double value, int index){
        this.armStates[index] = value;
    }

    public void zeroWrist(){
        wristEncoder.setPosition(0);
    }

    private double seekSpeed(double desiredPosition, double currentPosition) {
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

    public void armHold(double desiredPosition){

        if(Math.abs(armEncoder.getPosition() - desiredPosition) > deviation){
            this.armStates[0] = seekSpeed(desiredPosition,armEncoder.getPosition());
        } else {
            this.armStates[0] = 0;
        }
    }

    @Override
    public void periodic(){
        wristMove(this.armStates[1]);
        liftArm(this.armStates[0]);
        SmartDashboard.putNumber("armMotor1", armEncoder.getPosition());
        SmartDashboard.putNumber("wristMotor1", wristEncoder.getPosition());
        SmartDashboard.getNumber("High Position", HighPosition);
        SmartDashboard.getNumber("Middle Position", MidPosition);
        SmartDashboard.getNumber("Lower Position", LowPosition);
        SmartDashboard.getNumber("Stowed Position", StowPosition);
    }
}