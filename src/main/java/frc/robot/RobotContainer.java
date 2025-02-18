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
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

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
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class RobotContainer {

  public enum SIDES {
    SIDE1(new Pose2d()),
    SIDE2(new Pose2d()),
    SIDE3(new Pose2d()),
    SIDE4(new Pose2d()),
    SIDE5(new Pose2d()),
    SIDE6(new Pose2d());

    Pose2d pos;
    private SIDES(Pose2d pos){
        this.pos = pos;
    }

    public Pose2d getPose() {
      return pos;
    }
}

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

    // NamedCommands.registerCommand("Blank",Commands.none());
    NamedCommands.registerCommand("AlignSide1", Commands.sequence(Commands.print("ALIGNSIDE1"),new AutoAlign("limelight-driver", robotBase, 19)));
    NamedCommands.registerCommand("AlignSide2", Commands.sequence(Commands.print("ALIGNSIDE1"),new AutoAlign("limelight-driver", robotBase, 20)));
    
    // new EventTrigger("StopAlign").onTrue(Commands.sequence(new InstantCommand(() -> AutoAlign.idling = true), Commands.print("STOP")));
    // new EventTrigger("StartAlign").onTrue(Commands.sequence(new InstantCommand(() -> AutoAlign.idling = false), Commands.print("START")));

  }

  private void configureBindings() 
  {
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(3).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0,0));
    driverController.b.whileTrue(Commands.sequence(new InstantCommand(() -> AutoAlign.idling = false), new AutoAlign("limelight-driver", robotBase,19)));
    driverController.x.whileTrue(() -> System.out.println(robotBase.getCameraFacing(ReefPole.A.getTranslation()).config.table()));

    try {
      driverController.a.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Side1"), new PathConstraints(2, 1.5, 540, 720)));
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("LeftSide");
  }
}
