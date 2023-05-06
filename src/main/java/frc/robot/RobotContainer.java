// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kControllers;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  // Controllers
  private final CommandXboxController primaryController;
  private final CommandXboxController secondaryController;

  // Subsystems
  private final Drivetrain sys_drivetrain;

  // Commands
  private final DefaultDrive cmd_defaultDrive;

  public RobotContainer() {

    // Controllers
    primaryController = new CommandXboxController(kControllers.kPrimaryController);
    secondaryController = new CommandXboxController(kControllers.kSecondaryController);

    // Subsystems
    sys_drivetrain = new Drivetrain();

    // Commands
    cmd_defaultDrive = new DefaultDrive(sys_drivetrain, primaryController);

    sys_drivetrain.setDefaultCommand(cmd_defaultDrive);

    configureBindings();

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {

    return null;

  }
}
