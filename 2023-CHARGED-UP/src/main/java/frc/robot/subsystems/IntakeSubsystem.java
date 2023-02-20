package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_intakeMotor1 = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax m_intakeMotor2 = new CANSparkMax(17, MotorType.kBrushless);
    private double intakeSpeed = 0.20;
    private double intakeEject = 0.32;
    private Ultrasonic m_ultrasonic = new Ultrasonic(1,2);

    private MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_intakeMotor1, m_intakeMotor2);

    public IntakeSubsystem() {
        m_intakeMotor2.setInverted(true);
        intakeSpeed=SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        intakeEject=SmartDashboard.getNumber("Intake Eject", intakeEject);
        SmartDashboard.putNumber("Intake Eject", intakeEject);
    }

    public void intake(){

        // if(m_ultrasonic.getRangeInches()>2){
        //         m_intakeMotors.set(1);

        // } else{
        //         m_intakeMotors.set(0);
        // }
        m_intakeMotors.set(intakeSpeed);
    }


    public void intakeReverse()
    {
            m_intakeMotors.set(-intakeEject);
    }

    public void intakeStop() {
        m_intakeMotors.set(0);
    }

    @Override
    public void periodic()
    {
        intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
        intakeEject = SmartDashboard.getNumber("Intake Eject", intakeEject);
    }
}
