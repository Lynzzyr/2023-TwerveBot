// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;
import frc.robot.Constants.kRobot;

public class Drivetrain extends SubsystemBase {

  // Modules
  private final TwerveModule moduleFL;
  private final TwerveModule moduleFR;
  private final TwerveModule moduleBL;
  private final TwerveModule moduleBR;

  // Kinematic points
  private final Translation2d locFL;
  private final Translation2d locFR;
  private final Translation2d locBL;
  private final Translation2d locBR;

  // Sensors & location
  private final WPI_Pigeon2 gyro;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final Field2d field2d;

  public Drivetrain() {

    // Modules
    moduleFL = new TwerveModule(kDrivetrain.motorLeftID_FL, kDrivetrain.motorRightID_FL, kDrivetrain.driveEncID_FL, kDrivetrain.turnEncID_FL);
    moduleFR = new TwerveModule(kDrivetrain.motorLeftID_FR, kDrivetrain.motorRightID_FR, kDrivetrain.driveEncID_FR, kDrivetrain.turnEncID_FR);
    moduleBL = new TwerveModule(kDrivetrain.motorLeftID_BL, kDrivetrain.motorRightID_BL, kDrivetrain.driveEncID_BL, kDrivetrain.turnEncID_BL);
    moduleBR = new TwerveModule(kDrivetrain.motorLeftID_BR, kDrivetrain.motorRightID_BR, kDrivetrain.driveEncID_BR, kDrivetrain.turnEncID_BR);

    // Kinematic points
    locFL = new Translation2d(kRobot.kLength / 2, kRobot.kWidth / 2);
    locFR = new Translation2d(kRobot.kLength / 2, -kRobot.kWidth / 2);
    locBL = new Translation2d(-kRobot.kLength / 2, kRobot.kWidth / 2);
    locBR = new Translation2d(-kRobot.kLength / 2, -kRobot.kWidth / 2);

    // Sensors & location
    gyro = new WPI_Pigeon2(kDrivetrain.gyroID);
    zeroHeading();

    field2d = new Field2d();
    SmartDashboard.putData("Field", field2d);

    kinematics = new SwerveDriveKinematics(locFL, locFR, locBL, locBR);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
      new SwerveModulePosition[] {moduleFL.getPos(), moduleFR.getPos(), moduleBL.getPos(), moduleBR.getPos()}
    );

  }

  public void zeroHeading() {

    gyro.reset();

  }

  public Rotation2d getRotation2d() {

    return gyro.getRotation2d();

  }

  public double getHeadingDegrees() {

    return getRotation2d().getDegrees();

  }

  public void updateOdometry() {

    odometry.update(gyro.getRotation2d(),
      new SwerveModulePosition[] {moduleFL.getPos(), moduleFR.getPos(), moduleBL.getPos(), moduleBR.getPos()});

  }

  /* Note that axes labels have been differed from standard controller labels by the ChassisSpeeds class
   * 
   * xSpeed denotes forward/backward motion relative to field, controlled by the y-axis on a joystick
   * ySpeed denotes left/right motion relative to field, controlled by the x-axis on a joystick
   */
  public void drive(double xSpeed, double ySpeed, double rotation) {

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, gyro.getRotation2d())
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kDrivetrain.kMaxDriveVelocity);

    moduleFL.setDesiredState(moduleStates[0]);
    moduleFR.setDesiredState(moduleStates[1]);
    moduleBL.setDesiredState(moduleStates[2]);
    moduleBR.setDesiredState(moduleStates[3]);

  }

  public void haltMotors() {

    moduleFL.haltMotors();
    moduleFR.haltMotors();
    moduleBL.haltMotors();
    moduleBR.haltMotors();

  }

  @Override
  public void periodic() {

    updateOdometry();

    field2d.setRobotPose(odometry.getPoseMeters());

  }
}
