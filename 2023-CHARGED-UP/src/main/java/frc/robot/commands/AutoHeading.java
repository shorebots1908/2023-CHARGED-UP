package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoHeading extends CommandBase{
    private Swerve s_Swerve;
    private 

    public AutoHeading(Swerve swerve) {
        this.s_Swerve = swerve;

    }
}
