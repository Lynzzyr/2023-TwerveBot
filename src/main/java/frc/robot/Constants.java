// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class kControllers {

    public static final int kPrimaryController = 0;
    public static final int kSecondaryController = 1;

  }

  public static final class kRobot {

    // Placeholder values
    public static final double kLength = 1;
    public static final double kWidth = 1;

  }

  public static final class kDrivetrain {

    // Motor setup
    public static final int motorLeftID_FL = 1;
    public static final int motorLeftID_FR = 2;
    public static final int motorLeftID_BL = 3;
    public static final int motorLeftID_BR = 4;

    public static final int motorRightID_FL = 5;
    public static final int motorRightID_FR = 6;
    public static final int motorRightID_BL = 7;
    public static final int motorRightID_BR = 8;

    public static final int driveEncID_FL = 9;
    public static final int driveEncID_FR = 10;
    public static final int driveEncID_BL = 11;
    public static final int driveEncID_BR = 12;

    public static final int turnEncID_FL = 13;
    public static final int turnEncID_FR = 14;
    public static final int turnEncID_BL = 15;
    public static final int turnEncID_BR = 16;

    public static final int gyroID = 17;

    public static final int kMotorCurrentLimit = 30;

    // PID values (all placeholder)
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // Slewrate deadbands
    public static final double kXSpeedDeadband = 0.2;
    public static final double kYSpeedDeadband = 0.2;
    public static final double kRotationDeadband = 0.2;

    // Encoder & PID math and setup
    public static final double kWheelRadius = 0.025; // meters, estimated placeholder value
    public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;

    public static final double kMaxDriveVelocity = 4;
    public static final double kMaxDriveAngularVelocity = Math.PI; // half rotation
    public static final double kMaxTurnAngularAcceleration = 2 * Math.PI; // full rotation

    public static final int kEncoderTicksPerRevolution = 4096;
    public static final double kDriveEncoderCoefficient = kWheelCircumference / kEncoderTicksPerRevolution;
    public static final double kTurnEncoderCoefficient = 2 * Math.PI / kEncoderTicksPerRevolution;

    public static final String kDriveUnitString = "m";
    public static final String kTurnUnitString = "rad";

  }
}
