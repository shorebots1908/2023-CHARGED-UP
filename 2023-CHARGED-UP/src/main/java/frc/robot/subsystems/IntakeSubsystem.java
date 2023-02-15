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


public class IntakeSubsystem extends SubsystemBase {
    private PWMTalonFX m_intakeMotor1 = new PWMTalonFX(16);
    private PWMTalonFX m_intakeMotor2 = new PWMTalonFX(17);
  
    private Ultrasonic m_ultrasonic = new Ultrasonic(1,2);

    private MotorControllerGroup m_intakeMotors = new MotorControllerGroup(m_intakeMotor1, m_intakeMotor2);

    public IntakeSubsystem() {
        m_intakeMotor1.setInverted(true);
    }



    public void intake(){

        // if(m_ultrasonic.getRangeInches()>2){
        //         m_intakeMotors.set(1);

        // } else{
        //         m_intakeMotors.set(0);
        // }
        m_intakeMotors.set(0.32);
    }

    public void intakeReverse(){
            m_intakeMotors.set(-1);
    }
}
