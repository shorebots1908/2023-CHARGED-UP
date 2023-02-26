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
    public enum ArmJoint {
        Shoulder(0),
        Wrist(1);

        public final int value;

        ArmJoint(int value) {
            this.value = value;
        }
    }
    
    private double armSpeedLimit = 0.25;
    private double wristSpeedLimit = 0.25;
    private double[] armStates = {0.0, 0.0};

    //encoders
    private RelativeEncoder armEncoder;
    private RelativeEncoder wristEncoder;

    private double minSpeed = 0.05;
    private double maxSpeed = 1.0;

    //hold position settings
    //TODO: set proper values based on encoder readouts.
    private double motorRatios = 27.0 / 400.0;
    private double shoulderDeviation = 1;
    private double wristDeviation = 0.2;
    private double HighPosition = 50;
    private double MidPosition = 25;
    private double LowPosition = 0;
    private double StowPosition = 0;
    private double wristOffset = 0;
    private double currentHoldPosition;
    private double wristHoldPosition;
    private boolean armHolding = false;
    private boolean wristHolding = false;
    private double shoulderPosition;
    private double oldShoulderPosition;

    public boolean getWristHolding()
    {
        return wristHolding;
    }

    public void setWristHolding()
    {
        wristHolding = true;
    }

    public void unsetWristHolding()
    {
        wristHolding = false;
    }

    public boolean getArmHolding()
    {
        return armHolding;
    }

    public void setArmHolding()
    {
        armHolding = true;
    }
    
    public void unsetArmHolding()
    {
        armHolding = false;
    }

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
        oldShoulderPosition = armEncoder.getPosition();
    }

    public void armHoldSet(double desiredPosition) {
        currentHoldPosition = desiredPosition;
    }

    public double getHighPosition(){
        return HighPosition;
    }
    public double getMidPosition(){
        return MidPosition;
    }
    public double getLowPosition(){
        return LowPosition;
    }
    public double getStowPosition(){
        return StowPosition;
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

    public void wristUp(){ //Triggered by 'A' Button
        wristMove(0.1);
    }
    
    public void wristDown(){ //Triggered by 'B' Button
        wristMove(-0.1);
    }

    public void setWristHoldPosition(){ //Triggered by 'Y' Button Once
        wristHoldPosition = wristEncoder.getPosition();
    }

    public double getWristPosition(){
        return wristEncoder.getPosition();
    }

    public double getArmPosition(){
        return armEncoder.getPosition();
    }

    public void wristHold(){ //Triggered by 'Y' Button while Held
        if(Math.abs(wristEncoder.getPosition() - wristHoldPosition) > wristDeviation){
            this.armStates[1] = seekSpeed(wristHoldPosition, wristEncoder.getPosition());
        } else {
            this.armStates[1] = 0;
        }
    }

    public void setArmStates(double value, int index){
        this.armStates[index] = value;
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

    public void zeroWrist(){
        wristEncoder.setPosition(0);
    }

    public void armHold(double desiredPosition) {

        if(Math.abs(armEncoder.getPosition() - desiredPosition) > shoulderDeviation)
        {
            this.armStates[ArmJoint.Shoulder.value] = seekSpeed(desiredPosition, armEncoder.getPosition());
        }
        else
        {
            this.armStates[ArmJoint.Shoulder.value] = 0;
        }
    }

    public void wristHold(double desiredPosition) {

        if(Math.abs(wristEncoder.getPosition() - desiredPosition) > wristDeviation)
        {
            this.armStates[ArmJoint.Wrist.value] = seekSpeed(desiredPosition, wristEncoder.getPosition());
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
        shoulderPosition = armEncoder.getPosition();

        if(wristHolding)
        {
            wristHoldPosition += (shoulderPosition - oldShoulderPosition) * motorRatios;
            wristHold(wristHoldPosition);
        }
        wristMove(this.armStates[ArmJoint.Wrist.value]);

        if(armHolding)
        {
            armHold(currentHoldPosition);
        }
        liftArm(this.armStates[ArmJoint.Shoulder.value]);
        
        SmartDashboard.putNumber("armMotor1", armEncoder.getPosition());
        SmartDashboard.putNumber("wristMotor1", wristEncoder.getPosition());
        SmartDashboard.getNumber("High Position", HighPosition);
        SmartDashboard.getNumber("Middle Position", MidPosition);
        SmartDashboard.getNumber("Lower Position", LowPosition);
        SmartDashboard.getNumber("Stowed Position", StowPosition);
        SmartDashboard.putNumber("High Position", HighPosition);
        SmartDashboard.putNumber("Middle Position", MidPosition);
        SmartDashboard.putNumber("Lower Position", LowPosition);
        SmartDashboard.putNumber("Stowed Position", StowPosition);
        SmartDashboard.putBoolean("Arm Hold", armHolding);
        SmartDashboard.putBoolean("Wrist Holding", wristHolding);
        oldShoulderPosition = shoulderPosition;

    }
}