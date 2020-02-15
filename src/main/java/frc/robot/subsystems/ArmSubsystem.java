package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
private final SpeedController wrist = new WPI_TalonSRX(ArmConstants.armMotor);
private final SpeedController topIntake = new WPI_VictorSPX(ArmConstants.topIntake);
private final SpeedController bottomIntake = new WPI_VictorSPX(ArmConstants.bottomIntake);
private final DigitalInput hatch_ultrasonic1 = new DigitalInput(2);
private final DigitalInput hatch_ultrasonic2 = new DigitalInput(3);
private DigitalInput ball_sensor = new DigitalInput(4);
private DigitalInput wrist_limit = new DigitalInput(1);

 @Override
 public void periodic() {
     //This method will be called once per scheduler run
     SmartDashboard.putBoolean("wrist_limit", wrist_limit.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 1", hatch_ultrasonic1.get());
    SmartDashboard.putBoolean("hatch ultrasonic sensor 2", hatch_ultrasonic2.get());
    SmartDashboard.putBoolean("ball sensor",ball_sensor.get());
    int sensorPosition = ((WPI_TalonSRX)wrist).getSelectedSensorPosition();
    SmartDashboard.putNumber("armPosition", sensorPosition);
 }
}

/**
    //left bumper button extends the arm
    if (m_robotContainer.getController().getRawButton(5)) {
      armMotor.set(0.75);
    }
    //right bumper button retracts the arm
    else if (m_robotContainer.getController().getRawButton(6) && wrist_limit.get()) {
      armMotor.set(-0.75);
    }
    else {
      armMotor.set(0);
    }
    //Pressing down left joystick will intake
    if (m_robotContainer.getController().getRawButton(9)) {
      topIntake.set(-0.75);
      bottomIntake.set(-0.75);
    }
    //Pressing down right joystick will shoot
    else if (m_robotContainer.getController().getRawButton(10)) {
      topIntake.set(0.75);
      bottomIntake.set(0.75);
    }
    else {
      topIntake.set(0);
      bottomIntake.set(0);
    */