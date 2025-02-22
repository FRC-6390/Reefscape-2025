// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.security.PublicKey;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.commands.AutoCommands;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.DriveToPoint;
import frc.robot.commands.auto.PassiveAlign;
import frc.robot.commands.mechanisms.Elevate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class RobotContainer {

  public final LaserCan las = new LaserCan(59);
  public Elevator elevator = new Elevator();
  public Climber climber = new Climber();
  public EndEffector effector = new EndEffector();
  public Superstructure superstructure = new Superstructure(climber, elevator, effector);
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(3.5);
 
  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    // elevator.shuffleboard("Elevator");
    // climber.shuffleboard("Climber");
    // effector.shuffleboard("Effector");

    // NamedCommands.registerCommand("L4", superstructure.setElevator(ElevatorState.L4));
    // NamedCommands.registerCommand("L3", superstructure.setElevator(ElevatorState.L3));
    // NamedCommands.registerCommand("L2", superstructure.setElevator(ElevatorState.L2));
    // NamedCommands.registerCommand("L1", superstructure.setElevator(ElevatorState.L1));
    // NamedCommands.registerCommand("Feeder", superstructure.setElevator(ElevatorState.Feeder));
    // NamedCommands.registerCommand("Home", superstructure.setElevator(ElevatorState.Home));
    // NamedCommands.registerCommand("StartConfiguration", superstructure.setElevator(ElevatorState.StartConfiguration)); 
    
  }

  private void configureBindings() 
  {
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(3).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0,0));
    driverController.y.toggleOnTrue(new PassiveAlign(robotBase, las));
    // // driverController.a.onTrue(new DriveToPoint(robotBase , las));
    // // driverController.b.onTrue(new AutoAlign(robotBase.getCameraFacing(ReefPole.A.getTranslation()).config.table(),robotBase , las));
    // driverController.leftBumper.onTrue(new Elevate(ElevatorState.L4, las, superstructure, robotBase));
  }
  public Command getAutonomousCommand() {
    
    return new PathPlannerAuto("Choreo");
  }
}
