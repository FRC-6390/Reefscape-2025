// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import ca.frc6390.athena.controllers.DebouncedController;
import ca.frc6390.athena.core.RobotIMU;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.DriveToGoal;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {

  private final RobotIMU imu = RobotIMU.createFromPigeon2(Constants.DriveTrain.PIGEON_ID,  Constants.DriveTrain.CANBUS);
  private final RobotVision vision = new RobotVision(Constants.DriveTrain.LIMELIGHTS);
  public final DriveTrain driveTrain = new DriveTrain(imu);
 
  
  public final RobotLocalization localization = new RobotLocalization(driveTrain, vision, Constants.DriveTrain.LOCALIZATION_CONFIG);
  private final DebouncedController driverController = new DebouncedController(0);

  public RobotContainer() {

    
    configureBindings();
    
    driveTrain.setDriveCommand(driverController.leftX, driverController.leftY, driverController.rightX);
  }

  private void configureBindings() 
  {
    driverController.rightX.setDeadzone(Constants.Controllers.THETA_DEADZONE);
    driverController.leftX.setDeadzone(Constants.Controllers.THETA_DEADZONE);
    driverController.leftY.setDeadzone(Constants.Controllers.THETA_DEADZONE);

    driverController.start.onTrue(new InstantCommand(() -> driveTrain.getIMU().setYaw(0)));
    driverController.a.whileTrue(new AprilTagAlign(vision, localization, driveTrain, driverController));

    driverController.leftBumper.onTrue(new DriveToGoal(driveTrain, localization));
   
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
