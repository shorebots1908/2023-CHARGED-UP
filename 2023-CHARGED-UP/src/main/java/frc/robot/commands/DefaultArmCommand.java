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
        double liftInput = m_heightLiftRateSupplier.getAsDouble();
        if(liftInput > 0){
            m_ArmSubsystem.liftArm(liftInput);
        }
        else {
            m_ArmSubsystem.lowerArm(m_heightLowerRateSupplier.getAsDouble());
        }
        m_ArmSubsystem.wristMove(m_wristLiftRateSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.liftArm(0);
        m_ArmSubsystem.wristMove(0);
    }
}


