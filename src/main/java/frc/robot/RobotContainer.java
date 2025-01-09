// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.commands.SwerveDriveCommand;
import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotIMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  private final RobotIMU<?> imu = RobotIMU.createFromPigeon2(Constants.DriveTrain.PIGEON_ID,  Constants.DriveTrain.CANBUS);
  private final DriveTrain driveTrain = new DriveTrain(imu);
  private final DebouncedController driverController = new DebouncedController(0);

  public RobotContainer() {
    configureBindings();
    driveTrain.setDefaultCommand(new SwerveDriveCommand(driveTrain, driverController.leftX, driverController.leftY, driverController.rightX, Constants.DriveTrain.DRIVE_COMMAND_CONFIG));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
