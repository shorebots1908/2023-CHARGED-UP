// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Function;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final XboxController m_controller = new XboxController(0);
  private final CommandXboxController m_XBoxController = new CommandXboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  // private final JoystickButton zeroGyro = new JoystickButton(m_controller, XboxController.Button.kBack.value);
  private final JoystickButton robotCentric = new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value);

  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final Swerve s_Swerve = new Swerve();
  private int speedMode = 2;
  
  //TODO: Get wheels to rest in orientation. STill needed?
  //TODO: add slew rate to new swerve
  //TODO: account for gyroscope drift
  //TODO: use sensor to stop where pieces need to go
  //TODO: gyrostabilizationf
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation

    //AutoCommand definition
    final FunctionalCommand liftArm = new FunctionalCommand(
      null, 
      () -> {
        m_ArmSubsystem.armHold(0.65);
      }, 
      null, 
      m_ArmSubsystem::inPosition, 
      m_ArmSubsystem
      );

      final FunctionalCommand liftWrist = new FunctionalCommand(
        null, 
        () -> {
          m_ArmSubsystem.wristHold(0.65);
        }, 
        null, 
        m_ArmSubsystem::inPosition, 
        m_ArmSubsystem
        );

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> -m_controller.getRawAxis(translationAxis) * s_Swerve.speedScalar(speedMode), 
            () -> -m_controller.getRawAxis(strafeAxis) * s_Swerve.speedScalar(speedMode), 
            () -> -m_controller.getRawAxis(rotationAxis) * s_Swerve.speedScalar(speedMode), 
            () -> robotCentric.getAsBoolean()
        )
    );    


    m_ArmSubsystem.setDefaultCommand(new DefaultArmCommand(m_ArmSubsystem,
     () -> modifyAxis(m_controller.getRightTriggerAxis()),
     () -> modifyAxis(m_controller.getLeftTriggerAxis()),
     () -> modifyAxis(m_controller.getRightY())));

    // Configure the button bindings
    configureButtonBindings();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    m_XBoxController.back()
      .onTrue(Commands.runOnce(s_Swerve:: zeroGyro));
    m_XBoxController.rightBumper()
      .onTrue(Commands.runOnce(
        m_intakeSubsystem::intake, 
        m_intakeSubsystem));
    m_XBoxController.leftBumper()
      .whileTrue(Commands.startEnd(
        m_intakeSubsystem::intakeReverse,
        m_intakeSubsystem::intakeReverseRelease,
        m_intakeSubsystem));
    m_XBoxController.a()
      .onTrue(Commands.runOnce(() -> {
        m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getLowPosition(0));
        m_ArmSubsystem.wristHold(m_ArmSubsystem.getLowPosition(1));
        m_ArmSubsystem.setArmHolding();
      }));
    m_XBoxController.b()
      .onTrue(Commands.runOnce(() -> {
        m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getMidPosition(0));
        m_ArmSubsystem.wristHold(m_ArmSubsystem.getMidPosition(1));
        m_ArmSubsystem.setArmHolding();
      }));
    m_XBoxController.x()
      .onTrue(Commands.runOnce(() -> {
        m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getStowPosition(0));
        m_ArmSubsystem.wristHold(m_ArmSubsystem.getStowPosition(1));
        m_ArmSubsystem.setArmHolding();
      }));
    m_XBoxController.y()
      .onTrue(Commands.runOnce(() -> {
        m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getHighPosition(0));
        m_ArmSubsystem.wristHold(m_ArmSubsystem.getHighPosition(1));
        m_ArmSubsystem.setArmHolding();
      }));
      m_XBoxController.povUp()
        .onTrue(Commands.runOnce(() -> {speedMode = 0;}));
      m_XBoxController.povRight()
        .onTrue(Commands.runOnce(() -> {speedMode = 1;}));
      m_XBoxController.povDown()
        .onTrue(Commands.runOnce(() -> {speedMode = 2;}));
    m_XBoxController.rightStick()
        .onTrue(Commands.runOnce(s_Swerve::toggleOrientationMode));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
