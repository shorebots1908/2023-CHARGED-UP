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

		public Command Score() {

			var thetaController = 
				new ProfiledPIDController(
					Constants.AutoConstants.kPThetaController, 0 , 0, Constants.AutoConstants.kThetaControllerConstraints);
				thetaController.enableContinuousInput(-Math.PI, Math.PI);
			
				TrajectoryConfig config = 
				new TrajectoryConfig(
					Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.5, 
					Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
					.setKinematics(Constants.Swerve.swerveKinematics);
		
				TrajectoryConfig reverseConfig = 
					new TrajectoryConfig(
						Constants.AutoConstants.kMaxSpeedMetersPerSecond * 0.5, 
						Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.Swerve.swerveKinematics)
						.setReversed(true);
						
				Trajectory reverseTrajectory = 
				TrajectoryGenerator.generateTrajectory(
					List.of(new Pose2d(0, 0, Rotation2d.fromRadians(0)), 
					new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(0))), 
					reverseConfig);
							
				Trajectory advanceTrajectory = 
				TrajectoryGenerator.generateTrajectory(
					List.of(
						new Pose2d(0, 0, Rotation2d.fromRadians(0)), 
						new Pose2d(0, 0, Rotation2d.fromRadians(0))), 
						config);
		
				SwerveControllerCommand swerveControllerCommandAdvance = 
						new SwerveControllerCommand(
								advanceTrajectory, 
								c_Swerve::getPose, 
								Constants.Swerve.swerveKinematics, 
								
								new PIDController(Constants.AutoConstants.kPXController, 0, 0), 
								new PIDController(Constants.AutoConstants.kPXController, 0, 0), 
								thetaController,
								c_Swerve::setModuleStates,
								c_Swerve);
				
				InstantCommand advanceSetup = new InstantCommand(() -> {c_Swerve.resetOdometry(advanceTrajectory.getInitialPose());});
				InstantCommand reverseSetup = new InstantCommand(() -> {c_Swerve.resetOdometry(reverseTrajectory.getInitialPose());});



		}

}
