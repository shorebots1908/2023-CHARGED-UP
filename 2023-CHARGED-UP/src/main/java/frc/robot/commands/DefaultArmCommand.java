package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultArmCommand extends CommandBase {
    private final ArmSubsystem m_ArmSubsystem;
    private final DoubleSupplier m_heightLiftRateSupplier;
    private final DoubleSupplier m_heightLowerRateSupplier;

    public DefaultArmCommand(ArmSubsystem armSubsystem,
        DoubleSupplier heightLiftRateSupplier, 
        DoubleSupplier heightLowerRateSupplier) 
    {
        this.m_ArmSubsystem = armSubsystem;
        this.m_heightLiftRateSupplier = heightLiftRateSupplier;
        this.m_heightLowerRateSupplier = heightLowerRateSupplier;

        addRequirements(armSubsystem);
    }
    
    @Override
    public void execute() 
    {
        double liftInput = m_heightLiftRateSupplier.getAsDouble();
        if(liftInput > 0){
            m_ArmSubsystem.liftArm(liftInput);
        }
        else {
            m_ArmSubsystem.lowerArm(m_heightLowerRateSupplier.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.liftArm(0);
    }
}


