package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;





public class AutoMidCubeShot extends CommandBase {

private double changedPosition = armEncoder.setPosition(0.65);



public AutoMidCubeShot() {

}


public void midShot() {
    new FunctionalCommand(
    // Reset encoders on command start
    armEncoder.setPosition(0.65),
    // Start driving forward at the start of the command
    () -> ,
    // Stop driving at the end of the command
    interrupted -> armHoldSet(0.65),
    // End the command when the robot's driven distance exceeds the desired value
    () -> armEncoder.getArmPosition() >= changedPosition,
    // Require the drive subsystem
    m_ArmSubsystem
)
}











}