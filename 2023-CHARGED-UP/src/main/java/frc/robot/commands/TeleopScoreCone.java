package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

		public TeleopScoreCone(Swerve swerve, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, NetworkTable limelightTable) {
				c_Swerve = swerve;
				c_ArmSubsystem = armSubsystem;
				c_IntakeSubsystem = intakeSubsystem;
				c_limelightTable = limelightTable;

				addRequirements(swerve, armSubsystem, intakeSubsystem);
		}

		private final PIDController headingController = 
      new PIDController(Constants.AutoConstants.kPXController, 0, 0);
		
		private double rotation;
		private double targetX, targetY, targetArea;

		@Override
		public void execute()
		{
			rotation = headingController.calculate(0, 180);
			targetX = c_limelightTable.getValue("tx").getDouble();
			targetY = c_limelightTable.getValue("ty").getDouble();
			targetArea = c_limelightTable.getValue("ta").getDouble();
			if(c_limelightTable.getEntry("tv").getBoolean(false) && (c_Swerve.getYaw().getDegrees() > 170 || c_Swerve.getYaw().getDegrees() < -170))
			{
				Translation2d translation = new Translation2d(
					0.0, 
					Math.abs(targetX) > 1 ? Math.copySign(Math.min(Math.max(0.2, 0.2 * Math.abs(targetX)), 1.0), targetX) : 0
				);
				c_Swerve.drive(translation, rotation, true, true);
			}
		}

		@Override
		public void end(boolean interrupted)
		{

		}

		@Override
		public boolean isFinished()
		{
			return (Math.abs(targetX) < 1);
		}

}
