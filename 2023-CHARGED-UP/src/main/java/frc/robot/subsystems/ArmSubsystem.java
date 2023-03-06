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
    
    private double armSpeedLimit = 0.35;
    private double wristSpeedLimit = 0.35;
    private double[] armStates = {0.0, 0.0};

    //encoders
    private RelativeEncoder armEncoder;
    private RelativeEncoder armEncoder2;
    private RelativeEncoder wristEncoder;

    private double minSpeed = 0.05;
    private double maxSpeed = 1.0;

    //hold position settings
    //TODO: set proper values based on encoder readouts.
    private double motorRatios = 27.0 / 400.0;
    private double shoulderDeviation = 1;
    private double wristDeviation = 0.2;
    private double[] HighPosition = {137, 11.45};
    private double[] MidPosition = {120.8, 11};
    private double[] LowPosition = {24.7, 5.5};
    private double[] StowPosition = {0, 0};
    private double wristOffset = 0;
    private double currentHoldPosition;
    private double wristHoldPosition;
    private boolean armHolding = false;
    private boolean wristHolding = false;
    private double shoulderPosition1;
    private double shoulderPosition2;
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
        armEncoder2 = armMotor2.getEncoder();
        wristEncoder = wristMotor1.getEncoder();
        currentHoldPosition = armEncoder.getPosition();
        wristHoldPosition = wristEncoder.getPosition();
        StowPosition[0] = SmartDashboard.getNumber("Stowed Position", StowPosition[0]);
        HighPosition[0] = SmartDashboard.getNumber("High Position", HighPosition[0]);
        MidPosition[0] = SmartDashboard.getNumber("Middle Position", MidPosition[0]);
        LowPosition[0] = SmartDashboard.getNumber("Lower Position", LowPosition[0]);
        SmartDashboard.putNumber("High Position", HighPosition[0]);
        SmartDashboard.putNumber("Middle Position", MidPosition[0]);
        SmartDashboard.putNumber("Lower Position", LowPosition[0]);
        SmartDashboard.putNumber("Stowed Position", StowPosition[0]);

        wristOffset = SmartDashboard.getNumber("Wrist Offset", wristOffset);
        SmartDashboard.putNumber("Wrist Offset", wristOffset);
        oldShoulderPosition = armEncoder.getPosition();
    }

    public void armHoldSet(double desiredPosition) {
        currentHoldPosition = desiredPosition;
    }

    public boolean inPosition(){
        return seekSpeed(currentHoldPosition, armSpeedLimit) == 0;
    }
    public boolean isPosition(){
        return seekSpeed(if(Math.abs(wristEncoder.getPosition() - wristHoldPosition) > wristDeviation) {
            seekSpeed(wristHoldPosition, wristEncoder.getPosition());
        });
    }

    public double getHighPosition(int index){
        return HighPosition[index];
    }
    public double getMidPosition(int index){
        return MidPosition[index];
    }
    public double getLowPosition(int index){
        return LowPosition[index];
    }
    public double getStowPosition(int index){
        return StowPosition[index];
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

    public double getArmPosition2(){
        return armEncoder2.getPosition();
    }
    public void modifyWristHold(double addend){
    wristHoldPosition += addend;
    }

    public void wristHold(){ //Triggered by 'Y' Button while Held
        if(Math.abs(wristEncoder.getPosition() - wristHoldPosition) > wristDeviation){
            this.armStates[1] = wristSpeedLimit * seekSpeed(wristHoldPosition, wristEncoder.getPosition());
        } else {
            this.armStates[1] = 0;
        }
    }

    public void setArmStates(double value, int index){
        this.armStates[index] = value;
    }

    private double seekSpeed(double desiredPosition, double currentPosition) {
        double outputSpeed = (desiredPosition - currentPosition) * 0.3  ;
        
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
        double calculatedSpeed = seekSpeed(desiredPosition, armEncoder.getPosition());

        if(Math.abs(armEncoder.getPosition() - desiredPosition) > shoulderDeviation)
        {
            this.armStates[ArmJoint.Shoulder.value] = calculatedSpeed < armSpeedLimit ? calculatedSpeed : armSpeedLimit;
        }
        else
        {
            this.armStates[ArmJoint.Shoulder.value] = 0;
        }
    }

    public void wristHold(double desiredPosition) {
        double calculatedSpeed = seekSpeed(desiredPosition, wristEncoder.getPosition());

        if(Math.abs(wristEncoder.getPosition() - desiredPosition) > wristDeviation)
        {
            this.armStates[ArmJoint.Wrist.value] = calculatedSpeed < wristSpeedLimit ? calculatedSpeed : wristSpeedLimit;
        } 
        else 
        {
            this.armStates[ArmJoint.Wrist.value] = 0;
        }
    }

    
    @Override
    public void periodic() {
        shoulderPosition1 = armEncoder.getPosition();
        shoulderPosition2 = -armEncoder2.getPosition();

        if(wristHolding)
        {
            wristHoldPosition += (shoulderPosition1 - oldShoulderPosition) * motorRatios;
            wristHold(wristHoldPosition);
        }
        wristMove(this.armStates[ArmJoint.Wrist.value]);

        if(armHolding)
        {
            armHold(currentHoldPosition);
        }
        liftArm(this.armStates[ArmJoint.Shoulder.value]);

        SmartDashboard.putNumber("Wrist Setpoint", wristHoldPosition);
        SmartDashboard.putNumber("armMotor1", shoulderPosition1);
        SmartDashboard.putNumber("armMotor2", shoulderPosition2);
        SmartDashboard.putNumber("wristMotor1", wristEncoder.getPosition());
        SmartDashboard.putNumber("Shoulder Desync", Math.abs(shoulderPosition1 + shoulderPosition2));
        StowPosition[0] = SmartDashboard.getNumber("Stowed Position", StowPosition[0]);
        HighPosition[0] = SmartDashboard.getNumber("High Position", HighPosition[0]);
        MidPosition[0] = SmartDashboard.getNumber("Middle Position", MidPosition[0]);
        LowPosition[0] = SmartDashboard.getNumber("Lower Position", LowPosition[0]);
        SmartDashboard.putBoolean("Arm Hold", armHolding);
        SmartDashboard.putBoolean("Wrist Holding", wristHolding);
        oldShoulderPosition = shoulderPosition1;

    }
}