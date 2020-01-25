package org.ghrobotics.frc2020.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain extends SubsystemBase {

  private static final double kGearRatio = 7.29;
  private static final double kWheelRadiusInches = 3.0;
  double lspeed,rspeed;
  

  WPI_VictorSPX leftMaster = new WPI_VictorSPX(1);
  WPI_VictorSPX rightMaster = new WPI_VictorSPX(3);

  WPI_VictorSPX leftSlave = new WPI_VictorSPX(2);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(4);
  
  Encoder leftEncoder = new Encoder(0,1);
  Encoder rightEncoder = new Encoder(2,3);


  AHRS gyro = new AHRS(SPI.Port.kMXP);

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.3, 1.96, 0.06);

  PIDController leftPIDController = new PIDController(2.95, 0, 0);
  PIDController rightPIDController = new PIDController(2.95, 0, 0);

  Pose2d pose = new Pose2d();

  public Drivetrain() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /*public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60,
        rightEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60
    );
  }*/

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return pose;
  }
  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60, 
    rightEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60);
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    leftMaster.set(leftVolts / 12);
    rightMaster.set(rightVolts / 12);
  }

  public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
  }

  @Override
  public void periodic() {
   lspeed= leftEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
   rspeed = rightEncoder.getRate() / kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches) / 60;
    pose = odometry.update(getHeading(),lspeed,rspeed);
  }
}
