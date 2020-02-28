package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LemonLight extends SubsystemBase {
    public LemonLight(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
    } 
}