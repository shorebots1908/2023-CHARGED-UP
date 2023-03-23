package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LedLightsSubsystem extends SubsystemBase {
    private Spark ledStrip = new Spark(0);
    private double coneModeColor = 0.69; //yellow
    private double cubeModeColor = 0.91; //purple
    private double intakeColor = 0.77; //green
    // private ArmSubsystem m_ArmSubsystem;
    // private IntakeSubsystem m_IntakeSubsystem;
    
    public LedLightsSubsystem(/*ArmSubsystem armsubsystem, IntakeSubsystem intakesubsystem*/){
        // m_ArmSubsystem = armsubsystem;
        // m_IntakeSubsystem = intakesubsystem;
    }
    public void setLEDColor(double pwmColorCode) {
        ledStrip.set(pwmColorCode);
    }
    

    // @Override
    // public void periodic() {

    // }
}
