/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ArmConstants;

public class Wrist extends PIDSubsystem {

  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(4);

  public Wrist() {
    super(new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD));
    armMotor.configFactoryDefault();
    //This was the setup (probably wrong) before robot was command based
    //((WPI_TalonSRX)armMotor).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    getController().setTolerance(5);
    //TODO: set distance per pulse m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
    //setSetpoint(ShooterConstants.kShooterTargetRPS);
  }

  @Override
  public double getMeasurement() {
    double retValue = 0;  //TODO actually get measurement
    return retValue;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    armMotor.set(output);  //TODO: how to use the setpoint
  }

  @Override
  public void periodic() {
    int sensorPosition = armMotor.getSelectedSensorPosition();
    SmartDashboard.putNumber("armPosition", sensorPosition);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void move(double speed) {
    armMotor.set(speed);
  }

  public void stop() {
    armMotor.stopMotor();
  }

  // //The old one used to:
  //     //left bumper button extends the arm
  //     if (m_robotContainer.getController().getRawButton(5)) {
  //       armMotor.set(0.75);
  //     }
  //     //right bumper button retracts the arm
  //     else if (m_robotContainer.getController().getRawButton(6) && wrist_limit.get()) {
  //       armMotor.set(-0.75);
  //     }
  //     else {
  //       armMotor.set(0);
  //     }
}
