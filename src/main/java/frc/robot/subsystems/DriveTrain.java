/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  // The motors on the left side of the drive.
  private WPI_TalonSRX left1;
  private WPI_VictorSPX left2;
  private SpeedController leftMotors;

  // The motors on the right side of the drive.
  private WPI_TalonSRX right1;
  private WPI_VictorSPX right2;
  private SpeedController rightMotors;

  // The robot's drive
  private DifferentialDrive robotDrive;

  // The gyro sensor
  private AHRS navx;

  // Shifter Solenoid
  Solenoid shifter = new Solenoid(DriveConstants.kShifterSolenoid);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  public DriveTrain() {
    if(Robot.isReal()) {
      navx = new AHRS(SPI.Port.kMXP);

      left1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1);
      left2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2);

      right1 = new WPI_TalonSRX(DriveConstants.kRightMotor1);
      right2 = new WPI_VictorSPX(DriveConstants.kRightMotor2);

      left1.setInverted(true);
      left1.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.PIDIDX, 10);
      left1.setSensorPhase(false);
      left1.setNeutralMode(NeutralMode.Brake);

      left2.setInverted(true);
      left2.follow(left1);
      left2.setNeutralMode(NeutralMode.Brake);

      right1.setInverted(true);
      right1.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, DriveConstants.PIDIDX, 10);
      right1.setSensorPhase(true);
      right1.setNeutralMode(NeutralMode.Brake);

      right2.setInverted(true);
      right2.follow(right1);
      right2.setNeutralMode(NeutralMode.Brake);

      leftMotors = left1;
      rightMotors = right1;
    }else{
      leftMotors = new VictorSP(0);
      rightMotors = new VictorSP(1);
    }
    robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    zeroHeading();
    resetOdometry(new Pose2d());
  }

  public double getLeftPosition(){
    if(Robot.isReal())
      return left1.getSelectedSensorPosition(DriveConstants.PIDIDX) * DriveConstants.encoderConstant;
    else
      return 0;
  }

  public double getLeftVelocity(){
    if(Robot.isReal())
      return left1.getSelectedSensorVelocity(DriveConstants.PIDIDX) * DriveConstants.encoderConstant * 10;
    else
      return 0;
  }

  public double getRightPosition(){
    if(Robot.isReal())
      return right1.getSelectedSensorPosition(DriveConstants.PIDIDX) * DriveConstants.encoderConstant;
    else
      return 0;
  }

  public double getRightVelocity(){
    if(Robot.isReal())
      return right1.getSelectedSensorVelocity(DriveConstants.PIDIDX) * DriveConstants.encoderConstant * 10;
    else
      return 0;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftPosition(), getRightPosition());

    SmartDashboard.putNumber("Left Position", getLeftPosition());        
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Position", getRightPosition());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putString("Wheel Speeds", getWheelSpeeds().toString());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  
  public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
    robotDrive.arcadeDrive(forwardSpeed, rotationSpeed, true);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("Left Volts", leftVolts);
    SmartDashboard.putNumber("Right Volts", rightVolts);
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    robotDrive.feed();
  }

  public void resetEncoders() {
    if(Robot.isReal()) {
      left1.setSelectedSensorPosition(0, DriveConstants.PIDIDX, 10);
      right1.setSelectedSensorPosition(0, DriveConstants.PIDIDX, 10);
    }
  }

  public double getAverageEncoderDistance() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    if(Robot.isReal())
      navx.reset();
  }

  public double getHeading() {
    if(Robot.isReal())
      return Math.IEEEremainder(navx.getAngle(), 360) * -1;
    else
      return 0;
  }

  public double getTurnRate() {
    if(Robot.isReal())
      return navx.getRate() * -1;
    else
      return 0;
  }

}
