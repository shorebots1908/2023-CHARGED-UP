package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.*;

public class TeleopScoreCone extends CommandBase{
		private Swerve c_Swerve;
		private ArmSubsystem c_ArmSubsystem;
		private IntakeSubsystem c_IntakeSubsystem;
		private NetworkTable c_limelightTable;

		public TeleopScoreCone(Swerve swerve, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
				c_Swerve = swerve;
				c_ArmSubsystem = armSubsystem;
				c_IntakeSubsystem = intakeSubsystem;

				addRequirements(c_Swerve, c_ArmSubsystem, c_IntakeSubsystem);
		}

		private final PIDController headingController = 
      new PIDController(Constants.AutoConstants.kPXController, 0, 0);
		
		private double rotation;
		private double targetX, targetY, targetArea;
		private boolean isArmInPosition = false;

		@Override
		public void initialize()
		{
		}

		@Override
		public void execute()
		{
			rotation = headingController.calculate(0, 180);
			c_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
			targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
			targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
			targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
			SmartDashboard.putString(
				"Limelight Values", 
				"LLT: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false) + 
				" LLX: " + targetX + 
				" LLY: " + targetY + 
				" LLA: " + targetArea);
			if(c_limelightTable.getEntry("tv").getDouble(0) == 1 && (c_Swerve.getYaw().getDegrees() > 170 || c_Swerve.getYaw().getDegrees() < -170))
			{
				Translation2d translation = new Translation2d(
					0.0, 
					Math.abs(targetX) > 1 ? Math.copySign(Math.min(Math.max(0.2, 0.2 * Math.abs(targetX)), 1.0), targetX) : 0
				);
				c_Swerve.drive(translation, rotation, true, true);
			}
			else if(!(c_Swerve.getYaw().getDegrees() > 170 || c_Swerve.getYaw().getDegrees() < -170))
			{
				c_Swerve.drive(new Translation2d(), rotation, true, true);
			}
		}

		@Override
		public void end(boolean interrupted)
		{
			c_Swerve.drive(new Translation2d(), 0, true, true);
		}

		@Override
		public boolean isFinished()
		{
			return (Math.abs(targetX) < 1);
		}

}
