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
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.ArmConstants;

public class Wrist extends PIDSubsystem {

  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(4);
  private final DigitalInput wrist_limit = new DigitalInput(1);

  public Wrist() {
    super(new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD));
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    getController().setTolerance(5);
    armMotor.setSelectedSensorPosition(4018);

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
    SmartDashboard.putBoolean("wrist_limit", wrist_limit.get());
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

  public int getPosition() {
    return armMotor.getSelectedSensorPosition();
  }

  public boolean getLimit() {
    return wrist_limit.get();
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
