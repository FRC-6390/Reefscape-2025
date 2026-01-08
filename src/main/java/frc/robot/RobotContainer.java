// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Example;


public class RobotContainer {

  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  



  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(2);


  // public Elevate elevate = new Elevate(ElevatorState.Home, lasLeft, lasRight, superstructure, robotBase, elevator);
  public SendableChooser<Command> chooser;
  public RobotContainer() 
  {
    configureBindings();
   
NamedCommands.registerCommand("Weewoo", new InstantCommand( () -> System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")));

    // robotBase.getDrivetrain().setDriveCommand(driverController);
    // RobotAuto.registerPathplannerAuto("Rajveer");
    // RobotAuto.registerPathplannerAuto("Don't Kill Me");
    // RobotAuto.registerPathplannerAuto("Rookie Path");
    // RobotAuto.registerPathplannerAuto("Small Rookie");
    // RobotAuto.registerPathplannerAuto("Home");z
    // RobotAuto.registerPathplannerAuto("Example");
    // RobotAuto.registerPathplannerAuto("Reefscape");
    RobotAuto.registerPathplannerAuto("Better auto");
     
     chooser = RobotAuto.getSendableChooser("Better auto");
    
    SmartDashboard.putData(chooser);

    
  }

  private void configureBindings() 
  {
    driverController.start.onTrue(() -> robotBase.getIMU().setYaw(0)).after(2)
        .onTrue(() -> robotBase.getLocalization().resetFieldPose(0, 0, 0));
    
  }

  public Command getAutonomousCommand() 
  {
    
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
