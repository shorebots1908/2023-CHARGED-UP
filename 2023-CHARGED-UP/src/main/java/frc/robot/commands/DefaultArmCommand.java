package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends CommandBase {
    private final ArmSubsystem m_ArmSubsystem;
    private final DoubleSupplier m_heightLiftRateSupplier;
    private final DoubleSupplier m_heightLowerRateSupplier;
    private final DoubleSupplier m_wristLiftRateSupplier;

    public DefaultArmCommand(ArmSubsystem armSubsystem,
        DoubleSupplier heightLiftRateSupplier, 
        DoubleSupplier heightLowerRateSupplier,
        DoubleSupplier wristLiftRateSupplier) 
    {
        this.m_ArmSubsystem = armSubsystem;
        this.m_heightLiftRateSupplier = heightLiftRateSupplier;
        this.m_heightLowerRateSupplier = heightLowerRateSupplier;
        this.m_wristLiftRateSupplier = wristLiftRateSupplier;

        addRequirements(armSubsystem);
    }
    
    @Override
    public void execute() 
    {
        double armInput = m_heightLiftRateSupplier.getAsDouble();
        double wristInput = m_wristLiftRateSupplier.getAsDouble();

        if(wristInput != 0)
        {
            m_ArmSubsystem.setWristHolding(false);
            m_ArmSubsystem.setArmStates(m_wristLiftRateSupplier.getAsDouble(), 1);
        }
        else
        {
            m_ArmSubsystem.setWristHolding(true);
            m_ArmSubsystem.setWristHoldPosition();
        }

        if(armInput > 0)
        {
            m_ArmSubsystem.setArmHolding(false);
            m_ArmSubsystem.setArmStates(armInput, 0);
            m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getArmPosition());
        }
        else if(armInput < 0)
        {
            m_ArmSubsystem.setArmHolding(false);
            m_ArmSubsystem.setArmStates(-(m_heightLowerRateSupplier.getAsDouble()), 0);
            m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getArmPosition());
        }
        else
        {
            m_ArmSubsystem.setArmHolding(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setArmStates(0, 0);
        m_ArmSubsystem.setArmStates(0, 0);
    }
}


