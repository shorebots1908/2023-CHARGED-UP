package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DefaultArmCommand extends CommandBase {
    private final ArmSubsystem m_ArmSubsystem;
    private final DoubleSupplier m_heightLiftRateSupplier;
    private final DoubleSupplier m_heightLowerRateSupplier;
    private final DoubleSupplier m_wristLiftRateSupplier;
    private double time = 0;
    private double previousTime = 0;

    public DefaultArmCommand(ArmSubsystem armSubsystem,
        DoubleSupplier heightLiftRateSupplier, 
        DoubleSupplier heightLowerRateSupplier,
        DoubleSupplier wristLiftRateSupplier) 
    {
        this.m_ArmSubsystem = armSubsystem;
        this.m_heightLiftRateSupplier = heightLiftRateSupplier;
        this.m_heightLowerRateSupplier = heightLowerRateSupplier;
        this.m_wristLiftRateSupplier = wristLiftRateSupplier;
        
        time = Timer.getFPGATimestamp();

        addRequirements(armSubsystem);
    }
    
    @Override
    public void execute() 
    {
        double armInput = m_heightLiftRateSupplier.getAsDouble();
        double armInput2 = m_heightLowerRateSupplier.getAsDouble();
        double wristInput = m_wristLiftRateSupplier.getAsDouble();
        time = Timer.getFPGATimestamp();
        
        if(wristInput != 0)
        {
            // m_ArmSubsystem.unsetWristHolding();
            // m_ArmSubsystem.setArmStates(m_wristLiftRateSupplier.getAsDouble(), 1);
            // m_ArmSubsystem.setWristHoldPosition();
            m_ArmSubsystem.modifyWristHold(wristInput*(time - previousTime) * 0.5);
        }
        else
        {
            m_ArmSubsystem.setWristHolding();
        }

        if(armInput > 0)
        {
            m_ArmSubsystem.unsetArmHolding();
            m_ArmSubsystem.setArmStates(armInput, 0);
            m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getArmPosition());
        }
        else if(armInput2 > 0)
        {
            m_ArmSubsystem.unsetArmHolding();
            m_ArmSubsystem.setArmStates(-(m_heightLowerRateSupplier.getAsDouble()), 0);
            m_ArmSubsystem.armHoldSet(m_ArmSubsystem.getArmPosition());
        }
        else
        {
            m_ArmSubsystem.setArmHolding();
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setArmStates(0, 0);
        m_ArmSubsystem.setArmStates(0, 0);
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   

