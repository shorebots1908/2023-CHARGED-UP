package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;

public class AutoTranslate extends CommandBase {

    private Swerve s_Swerve;
    private double x_Speed;
    private double y_Speed;
    private double duration;
    private double startTime;

    public AutoTranslate(Swerve swerve, double x, double y, double t) {
        s_Swerve = swerve;
        addRequirements(swerve);

        x_Speed = x;
        y_Speed = y;
        duration = t;
    }
 
    @Override
    public void execute() {
        startTime = Timer.getFPGATimestamp();
        if(Timer.getFPGATimestamp() - startTime <= duration) {
            s_Swerve.drive(new Translation2d(x_Speed, y_Speed), 0, true, false);

        }
        
    }
    

}