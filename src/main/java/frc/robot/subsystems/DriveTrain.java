package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.kLeftMotor1);
  private final WPI_VictorSPX leftRear = new WPI_VictorSPX(DriveConstants.kLeftMotor2);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.kRightMotor1);
  private final WPI_VictorSPX rightRear = new WPI_VictorSPX(DriveConstants.kRightMotor2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftFront, rightFront);

  public DriveTrain() {
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    m_robotDrive.arcadeDrive(forwardSpeed, rotationSpeed);
  }
}
