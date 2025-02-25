// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  
  private final Translation2d m_frontLeftLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);

  
  private final SwerveModule m_frontLeft = new SwerveModule(12, 11, 13, Constants.Swerve.Modules.Mod0.constants,false);
  
  private final SwerveModule m_frontRight = new SwerveModule(22, 21, 23, Constants.Swerve.Modules.Mod1.constants,true);
  private final SwerveModule m_backLeft = new SwerveModule(32, 31, 33, Constants.Swerve.Modules.Mod2.constants,true);
  private final SwerveModule m_backRight = new SwerveModule(42, 41, 43, Constants.Swerve.Modules.Mod3.constants,false);



  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation,
          m_frontRightLocation,
          m_backLeftLocation,
          m_backRightLocation
        );

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Mod 0 Cancoder" , m_frontLeft.getCANcoderAngle().getDegrees());
    SmartDashboard.putNumber("Mod 1 Cancoder" , m_frontRight.getCANcoderAngle().getDegrees());
    SmartDashboard.putNumber("Mod 2 Cancoder" , m_backLeft.getCANcoderAngle().getDegrees());
    SmartDashboard.putNumber("Mod 3 Cancoder" , m_backRight.getCANcoderAngle().getDegrees());

    m_backLeft.getCANcoderAngle();
  }

//45,135,180,225
  public void wheelsIn() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0.25, Rotation2d.fromDegrees(0)), "frontLeft", false);
    m_frontRight.setDesiredState(new SwerveModuleState(0.25, Rotation2d.fromDegrees(0)), "frontRight", false);
    m_backLeft.setDesiredState(new SwerveModuleState(0.25, Rotation2d.fromDegrees(0)), "backRight", false);
    m_backRight.setDesiredState(new SwerveModuleState(0.25, Rotation2d.fromDegrees(0)), "backLeft", false);
   }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

  public  double frontleft(){
    return m_frontLeft.getCANcoderAngle().getDegrees();
  }

  public double backleft(){
    return m_backLeft.getCANcoderAngle().getDegrees();
  }

  public  double frontright(){
    return m_frontRight.getCANcoderAngle().getDegrees();
  }

  public  double backright(){
    return m_backRight.getCANcoderAngle().getDegrees();
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {

    ChassisSpeeds desiredChassisSpeeds =
      fieldRelative ?
      ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed,
              ySpeed,
              rot,
              m_gyro.getRotation2d()
      )
      : new ChassisSpeeds(
              xSpeed,
              ySpeed,
              rot
      );
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    System.out.println("desired chassis: " + desiredChassisSpeeds);
    //SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    /*
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    */

    //System.out.println(">>>>>> SwerveModuleState (frontLeft): " + swerveModuleStates[0] +"\r\n");
    //System.out.println(">>>>>> SwerveModuleState (frontRight): " + swerveModuleStates[1] +"\r\n");

    m_frontLeft.setDesiredState(swerveModuleStates[0], "frontLeft", true);
    m_frontRight.setDesiredState(swerveModuleStates[1], "frontRight", true);
    m_backLeft.setDesiredState(swerveModuleStates[2],"backLeft", true);
    m_backRight.setDesiredState(swerveModuleStates[3],"backRight", true);
    
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    var posData_m_frontLeft = m_frontLeft.getPosition();
    var posData_m_frontRight = m_frontRight.getPosition();
    var posData_m_backLeft = m_backLeft.getPosition();
    var posData_m_backRight = m_backRight.getPosition();

    //System.out.println(">>>>>> CANCoder Position (frontLeft): " + posData_m_frontLeft.toString() + "\r\n");
    //System.out.println(">>>>>> CANCoder Position (frontRight): " + posData_m_frontRight.toString() + "\r\n");
    
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          posData_m_frontLeft,
          posData_m_frontRight,
          posData_m_backLeft,
          posData_m_backRight
        });
  }
}
