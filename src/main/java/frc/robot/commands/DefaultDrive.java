// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final CommandXboxController controller;

  private final SlewRateLimiter xSpeedSrl;
  private final SlewRateLimiter ySpeedSrl;
  private final SlewRateLimiter rotationSrl;

  public DefaultDrive(Drivetrain drivetrain, CommandXboxController controller) {

    this.drivetrain = drivetrain;
    this.controller = controller;

    xSpeedSrl = new SlewRateLimiter(kDrivetrain.kMaxDriveVelocity);
    ySpeedSrl = new SlewRateLimiter(kDrivetrain.kMaxDriveVelocity);
    rotationSrl = new SlewRateLimiter(kDrivetrain.kMaxDriveVelocity);

    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double xSpeed = controller.getLeftY();
    double ySpeed = controller.getLeftX();
    double rotation = controller.getRightX();

    xSpeed = xSpeedSrl.calculate(MathUtil.applyDeadband(xSpeed, kDrivetrain.kXSpeedDeadband)) * kDrivetrain.kMaxDriveVelocity;
    ySpeed = ySpeedSrl.calculate(MathUtil.applyDeadband(ySpeed, kDrivetrain.kYSpeedDeadband)) * kDrivetrain.kMaxDriveVelocity;
    rotation = rotationSrl.calculate(MathUtil.applyDeadband(rotation, kDrivetrain.kRotationDeadband)) * kDrivetrain.kMaxDriveAngularVelocity;

    drivetrain.drive(xSpeed, ySpeed, rotation);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return false;
    
  }
}
