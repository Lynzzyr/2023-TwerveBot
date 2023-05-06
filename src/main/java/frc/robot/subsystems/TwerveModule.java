// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDrivetrain;

/* Custom swerve module with tank treads
 * NEO motors, 6.75:1 gear ratio
 */

public class TwerveModule extends SubsystemBase {

  // Motors & drive
  private final CANSparkMax motorLeft;
  private final CANSparkMax motorRight;

  private final DifferentialDrive drive;

  // Encoders
  private final WPI_CANCoder driveEncoder; // Assumes that the encoder is placed so that it only rotates when the DifferentialDrive moves straight and is not rotating
  private final CANCoderConfiguration driveEncoderConfig;
  private final WPI_CANCoder turnEncoder;
  private final CANCoderConfiguration turnEncoderConfig;

  // PID
  private final PIDController drivePID;
  private final ProfiledPIDController turnPID;

  public TwerveModule(int motorLeftID, int motorRightID, int driveEncoderID, int turnEncoderID) {
    
    // Motors & drive
    motorLeft = new CANSparkMax(motorLeftID, MotorType.kBrushless);
    motorRight = new CANSparkMax(motorRightID, MotorType.kBrushed);

    motorLeft.restoreFactoryDefaults();
    motorLeft.setIdleMode(IdleMode.kBrake);
    motorLeft.setSmartCurrentLimit(kDrivetrain.kMotorCurrentLimit);
    motorLeft.burnFlash();
    
    motorRight.restoreFactoryDefaults();
    motorRight.setIdleMode(IdleMode.kBrake);
    motorRight.setInverted(true);
    motorRight.setSmartCurrentLimit(kDrivetrain.kMotorCurrentLimit);
    motorRight.burnFlash();

    drive = new DifferentialDrive(motorLeft, motorRight);

    // Encoders
    driveEncoder = new WPI_CANCoder(driveEncoderID);
    driveEncoderConfig = new CANCoderConfiguration();
    turnEncoder = new WPI_CANCoder(turnEncoderID);
    turnEncoderConfig = new CANCoderConfiguration();

    driveEncoderConfig.sensorCoefficient = kDrivetrain.kDriveEncoderCoefficient;
    driveEncoderConfig.unitString = kDrivetrain.kDriveUnitString;
    driveEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    driveEncoder.configAllSettings(driveEncoderConfig);
    
    turnEncoderConfig.sensorCoefficient = kDrivetrain.kTurnEncoderCoefficient;
    turnEncoderConfig.unitString = kDrivetrain.kTurnUnitString;
    turnEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    turnEncoder.configAllSettings(turnEncoderConfig);

    resetEncoders();

    // PID
    drivePID = new PIDController(kDrivetrain.kP, kDrivetrain.kI, kDrivetrain.kD);
    turnPID = new ProfiledPIDController(kDrivetrain.kP, kDrivetrain.kI, kDrivetrain.kD, 
      new TrapezoidProfile.Constraints(kDrivetrain.kMaxDriveAngularVelocity, kDrivetrain.kMaxTurnAngularAcceleration)
    );
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

  }

  public void haltMotors() {

    motorLeft.set(0);
    motorRight.set(0);

  }

  public void resetEncoders() {

    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);

  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));

  }

  public SwerveModulePosition getPos() {

    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));

  }

  public void setDesiredState(SwerveModuleState desiredState) {

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.005) {

      haltMotors();
      return;

    }

    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnEncoder.getPosition()));

    drive.arcadeDrive(drivePID.calculate(driveEncoder.getPosition(), optimizedState.speedMetersPerSecond) / 12,
      turnPID.calculate(turnEncoder.getPosition(), optimizedState.angle.getRadians()) / 12
    );

  }

  @Override
  public void periodic() {}
}
