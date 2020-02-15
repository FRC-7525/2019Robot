package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

//Arm Command
public class WristComand extends CommandBase {
    private final ArmSubsystem m_ArmSubsystem;
    private final DoubleSupplier m_wrist;

//Creates a new WristComand
public WristComand(ArmSubsystem m_ArmSubsystem, DoubleSupplier m_wrist) {
    m_ArmSubsystem = ArmSubsystem;
    m_wrist = wrist;
    addRequirements(ArmSubsystem);
 }
 @Override
 public void execute() {

 }
 @Override
 public boolean isFinished() {
     return false;
 }
}