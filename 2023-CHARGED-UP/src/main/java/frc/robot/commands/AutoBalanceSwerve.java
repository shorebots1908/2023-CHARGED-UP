package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.Swerve;

/**
 * AutoBalanceSwerve
 * Moved forward and back depending on gyro tilt/pitch inorder to keep robot on a tilting platform
 * @author FRC Team 1731
 * @url https://github.com/team1731/FRC2023/blob/main/ROBOT_CODE/AlexanderGCowbell/src/main/java/frc/robot/commands/AutoBalanceSwerve.java
 * 
 * Suggested we try there code out so here it is. Thanks guys!
 * 
 */
public class AutoBalanceSwerve extends CommandBase{


    private Double desiredHeading = 0.0;
    private boolean fieldrelative = true;
    private boolean openLoop = false;
    private boolean isFinished = false;

    
    private final PIDController headingController = 
        new PIDController(Constants.AutoConstants.kPXController, 0, 0);
       

    private double rotation;
    private Translation2d translation = new Translation2d(0 , 0);

    
    private Swerve s_Swerve;

    /**
     * Driver control
     */
    public AutoBalanceSwerve(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);  
    }
  
    @Override
    public void initialize() {
        isFinished = false;
        //s_Swerve.setLockWheels(true);
    }
    @Override
    public void execute() {

 

        rotation = headingController.calculate(s_Swerve.getYaw().getDegrees(), desiredHeading);  
        if (s_Swerve.getPitch() > 9) {
            translation = new Translation2d(-0.3, 0.0); // Speed is in Meters/s
        } else if (s_Swerve.getPitch() < -9) {
            translation = new Translation2d(0.3, 0);
        } else {
            translation = new Translation2d(0 , 0);
            if(Timer.getMatchTime() <= 0.1) {
                //s_Swerve.setLockWheels(false);
                isFinished = true;
            }
        }

        s_Swerve.drive(translation, rotation, fieldrelative, openLoop);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean innterruped) {
        //s_Swerve.setLockWheels(false);
        isFinished = true;
    }
}