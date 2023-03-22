package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.button.Button;
// import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_intakeMotor1 = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax m_intakeMotor2 = new CANSparkMax(17, MotorType.kBrushless);
    private double intakeSpeed = 0.80;
    private double intakeEject = 0.6;
    private boolean runIntake = false;
    private boolean runReverse = false;
    private RelativeEncoder encoder1, encoder2;
    private Ultrasonic m_ultrasonic = new Ultrasonic(1,2);
    private double timeCheck;
    private double minRPM = 0.3;
    private boolean cubeMode = false;
    private boolean coneMode = true;

    private LedLightsSubsystem ledLightsSubsystem;
    private double coneModeColor = 0.69; //yellow
    private double cubeModeColor = 0.91; //purple
    private double intakeColor = 0.77; //green

    ///swap the ports on the solenoids (forwardChannel and reverseChannel) if the piston is going the wrong way.
    private DoubleSolenoid solenoid1 = new DoubleSolenoid(18, PneumaticsModuleType.CTREPCM, 0, 1);

    private MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_intakeMotor1, m_intakeMotor2);

    public IntakeSubsystem(LedLightsSubsystem ledLightsSubsystem) {
        m_intakeMotor1.setIdleMode(IdleMode.kBrake);
        m_intakeMotor2.setIdleMode(IdleMode.kBrake);
        m_intakeMotor2.setInverted(true);
        intakeSpeed=SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        intakeEject=SmartDashboard.getNumber("Intake Eject", intakeEject);
        SmartDashboard.putNumber("Intake Eject", intakeEject);
        m_intakeMotor1.setSmartCurrentLimit(20);
        m_intakeMotor2.setSmartCurrentLimit(20);
        encoder1 = m_intakeMotor1.getEncoder();
        encoder2 = m_intakeMotor2.getEncoder();
        solenoid1.set(Value.kForward);
        this.ledLightsSubsystem = ledLightsSubsystem;
    }
    
    public void setCubeMode(){
        cubeMode = true;
        coneMode = false;
        solenoid1.set(Value.kReverse);
    }
    public void setConeMode(){
        cubeMode = false;
        coneMode = true;
        solenoid1.set(Value.kForward);
    }   

    public void intakeStop() {
        runIntake = false;
        m_intakeMotors.set(0);
    }
    
    public void intakeHold() {
        m_intakeMotors.set(0.1);
    }
    
    public void intake(){
        runIntake = !runIntake;
        if(runIntake) {
            timeCheck = Timer.getFPGATimestamp();
        }
    }
    
    
    public void intakeReverse()
    {
        runIntake = false;
        runReverse = true;
        m_intakeMotors.set(-intakeEject);
        if(coneMode)
        {
            solenoid1.set(Value.kReverse);
        }
    }
    
    public void intakeReverseRelease() {
        runReverse = false;
    }
    
    @Override
    public void periodic()
    {
        if(runIntake){
            m_intakeMotors.set(intakeSpeed);
            ledLightsSubsystem.setLEDColor(intakeColor);
            if(Timer.getFPGATimestamp() - timeCheck > 0.5){//Hold intake motor at low speed
                if(Math.abs(encoder1.getVelocity()) < minRPM || Math.abs(encoder2.getVelocity()) < minRPM) {
                    if(Timer.getFPGATimestamp() - timeCheck > 40){ //Stop after 60 seconds
                        intakeStop();
                    }
                    else if(cubeMode) {
                        intakeHold(); 
                    }
                    else if(coneMode){
                        intakeStop();
                    }
                }
            }
        }
        else {
            //TODO: add functionality to pull piece back in if the intake motors are letting it slip out. 
            if(cubeMode)
            {
                ledLightsSubsystem.setLEDColor(cubeModeColor);
            }
            else if(coneMode)
            {
                ledLightsSubsystem.setLEDColor(coneModeColor);
            }
            
            if(!runReverse){
                intakeStop();
            }
        }
        intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        intakeEject = SmartDashboard.getNumber("Intake Eject", intakeEject);
        SmartDashboard.putBoolean("Intake Active", runIntake);
        SmartDashboard.putNumber("Intake Motor 1", encoder1.getVelocity());
        SmartDashboard.putNumber("Intake Motor 2", encoder2.getVelocity());
    }
}
