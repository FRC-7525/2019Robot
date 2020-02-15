/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

  import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private final SpeedController left1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1);
  private final SpeedController left2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2);
  private final SpeedController right1 = new WPI_TalonSRX(DriveConstants.kRightMotor1);
  private final SpeedController right2 = new WPI_VictorSPX(DriveConstants.kRightMotor2);

  private final SpeedControllerGroup m_leftControllerGroup = new SpeedControllerGroup(left1, left2);
  private final SpeedControllerGroup m_rightControllerGroup = new SpeedControllerGroup(right1, right2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  public DriveTrain() {
    left1.setInverted(true);
    left2.setInverted(true);
    right1.setInverted(true);
    right2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    m_robotDrive.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}
