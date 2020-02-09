/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.kLeftFrontMotor);
  private final WPI_TalonSRX leftRear = new WPI_TalonSRX(DriveConstants.kLeftRearMotor);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.kRightFrontMotor);
  private final WPI_TalonSRX rightRear = new WPI_TalonSRX(DriveConstants.kRightRearMotor);

  private final SpeedControllerGroup m_leftControllerGroup = new SpeedControllerGroup(leftFront, leftRear);
  private final SpeedControllerGroup m_rightControllerGroup = new SpeedControllerGroup(rightFront, rightRear);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  public DriveTrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    m_robotDrive.arcadeDrive(forwardSpeed * -1, rotationSpeed * -1);
  }
}
