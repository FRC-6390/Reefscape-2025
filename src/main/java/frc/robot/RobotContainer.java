// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import ca.frc6390.athena.commands.AutoCommands;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.ALIGNMODE;
// import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.Climb;
// import frc.robot.commands.AprilTagAlign.ALIGNMODE;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;

public class RobotContainer {

  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  // public final Climber climber = new Climber();
  // public final Elevator elevator = new Elevator();
  // public final Elevator elevator = new Elevator();
  // public final Superstructure superstructure = new Superstructure(climber);
  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(3.5);
  // public RobotLocalization localization = robotBase.getLocalization();
  public RobotContainer() {
    configureBindings();
    driverController.leftY.setInverted(false);
    robotBase.getDrivetrain().setDriveCommand(driverController);

    // AutoCommands.registerCommand("ElevatorState.L1", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L1), elevator);
    // AutoCommands.registerCommand("ElevatorState.L2", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L2), elevator);
    // AutoCommands.registerCommand("ElevatorState.L3", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L3), elevator);
    // AutoCommands.registerCommand("ElevatorState.L4", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L4), elevator);
    // AutoCommands.registerCommand("ElevatorState.Home", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.Home), elevator);
    // AutoCommands.registerCommand("ElevatorState.Feeder", () -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.Feeder), elevator);

    // elevator.shuffleboard("Elevator");
    NamedCommands.registerCommand("Align", new AutoAlign(robotBase.getVision().getCamera("limelight-driver"), robotBase.getDrivetrain(), driverController,ALIGNMODE.REEF, robotBase.getLocalization()));
    // NamedCommands.registerCommand("AlignFeeder", new AprilTagAlign(robotBase.getVision().getCamera("limelight-tag"), robotBase.getDrivetrain(), driverController, ALIGNMODE.FEEDER));
   
  }

  private void configureBindings() 
  {

    driverController.start.onTrue(new InstantCommand(() -> robotBase.getDrivetrain().getIMU().setYaw(0)));
    driverController.b.whileTrue(new AutoAlign(robotBase.getVision().getCamera("limelight-driver"), robotBase.getDrivetrain(), driverController, frc.robot.commands.AutoAlign.ALIGNMODE.REEF, robotBase.getLocalization()));

    // driverController.rightBumper.whileTrue(() -> elevator.setMotors(-0.05)).onFalse(elevator::stop);
    // driverController.leftBumper.whileTrue(() -> elevator.setMotors(0.1)).onFalse(elevator::stop);


    // driverController.a.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L1));
    // driverController.b.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L2));
    // driverController.x.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L3));
    // driverController.y.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.L4));
    // driverController.rightBumper.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.Home));
    // driverController.leftBumper.onTrue(() -> elevator.getStateMachine().setGoalState(Elevator.ElevatorState.Feeder));
    //35.7    

    // driverController.leftBumper.onTrue(new Climb(climber, STATE.HOME));
    // driverController.rightBumper.whileTrue(new Climb(climber, STATE.CLIMB));
    // driverController.leftBumper.whileTrue(superstructure.setClimber(Climber.State.Home));
    // driverController.rightBumper.whileTrue(superstructure.setClimber(Climber.State.Climb));
  // ChassisSpeeds.fromFieldRelativeSpeeds(speeds, null);
  }
  public Command getAutonomousCommand() {
    return Commands.none();
    // return new PathPlannerAuto("New Auto");
    // PathPlannerPath exampleChoreoTraj = null;
    // try {
    //   exampleChoreoTraj = PathPlannerPath.fromChoreoTrajectory("R");
    //   robotBase.getLocalization().resetFieldPose(exampleChoreoTraj.getStartingHolonomicPose().get());
    // } catch (FileVersionException | IOException | ParseException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // return AutoBuilder.followPath(exampleChoreoTraj);
  }
}
