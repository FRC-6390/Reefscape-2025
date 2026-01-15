// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import javax.sound.midi.SysexMessage;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.Elevator.S;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;
import frc.robot.utils.Aiming.Interpolator;
import frc.robot.utils.Aiming.Loggable;
import frc.robot.utils.Aiming.ShotSolver;
import frc.robot.utils.Align.AlignCamera;
import frc.robot.utils.Align.AutoAling;
import frc.robot.utils.Align.AlignHelper.AlignMode;
import frc.robot.utils.DashboardConfiguration.ArmCalibrator;
import frc.robot.utils.DashboardConfiguration.ArmCalibrator.DashboardType;
import frc.robot.utils.Align.AlignHelper;
import frc.robot.utils.Experimental.ActionableConstraint;
import frc.robot.utils.Experimental.Constraint;
import frc.robot.utils.Experimental.DigitalSensor;
import frc.robot.utils.Experimental.SuperStructureStates;
import frc.robot.utils.Experimental.SuperStructureTest;
import frc.robot.utils.Experimental.SuperstructureBuilder;


public class RobotContainer {
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
 
  public AlignCamera camLeft = new AlignCamera(robotBase.getVision().getLimelight("limelight-left"), -Units.inchesToMeters(0.5), -Units.inchesToMeters(9.25), 15, 0);
  public AlignCamera camRight = new AlignCamera(robotBase.getVision().getLimelight("limelight-right"), -Units.inchesToMeters(0.5), Units.inchesToMeters(9.25), -15, 0);
  

  public StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build().shuffleboard("Arm", SendableLevel.DEBUG);
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build().shuffleboard("Wrist", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.CORAL_ROLLERS.build().shuffleboard("Rollers", SendableLevel.COMP);
  public final StatefulMechanism<RollerState> algaeRollers = Constants.EndEffector.ALGAE_ROLLERS.build().shuffleboard("Algae Rollers", SendableLevel.COMP);;

  public SuperStructureTest<SuperStructureStates> s = SuperstructureBuilder.builder()
                                                            .addArms(arm, wrist).addMotors(rollers, algaeRollers)
                                                            .addSensors(new DigitalSensor("Intake", new DigitalInput(4), true))
                                                            .build();

  public ArmCalibrator calibrator;
  public PIDController rController = new PIDController(0.11, 0, 0);


  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(1, 0, 0), 
                                                          new PIDController(1, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));


  
  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(0.15)
                                                              .setLeftSlewrate(1)
                                                              ;

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 
  public SendableChooser<Command> chooser;
  public Loggable logger;

  public AlignHelper shotTracker = new AlignHelper(
        10,
        new Pose2d(-Units.inchesToMeters(10), 0, new Rotation2d()),
        robotBase, 
        rController, 
        controller, 
        AlignMode.LOOKAT,
        camLeft, camRight
      );

  public AlignHelper climbTracker = new AlignHelper(
        15,
        new Pose2d(Units.inchesToMeters(20), -Units.inchesToMeters(10), new Rotation2d()),
        robotBase, 
        rController, 
        controller, 
        AlignMode.PARALLEL,
        camLeft, camRight
      );



  


  public double dist = 0;
  public double armSupplier = -92d;
  public boolean isAiming = false;

  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    robotBase.registerMechanism(arm, algaeRollers, wrist, rollers);
    robotBase.getLocalization().setSuppressUpdates(false);
    arm.setPidEnabled(true);
    wrist.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setFeedforwardEnabled(false);
    shotTracker.init();
    climbTracker.init();

    
    s.addActionableConstraint(new ActionableConstraint<SuperStructureStates>(SuperStructureStates.Intaking,SuperStructureStates.Score, () -> !s.getSensor("Intake").getSensorStatus()));
    s.addUpdateEvent(() -> 
      {
        shotTracker.setRelativePose();
        climbTracker.setRelativePose();
        shotTracker.shuffleboard();
        climbTracker.shuffleboard();

        armSupplier = 
            ShotSolver.computeAngleInDegrees(
              Units.inchesToMeters(10), 
              Units.inchesToMeters(20), 
              shotTracker.getDistanceToTarget(),
              rollers.getVelocity(), 
              Units.inchesToMeters(2)
            );

        //turrentSupplier = shotTracker.getAngleToTarget().getDegrees();

        if(isAiming)
        {
          ChassisSpeeds speeds = robotBase.getRobotSpeeds().getSpeeds("feedback");
          robotBase.getRobotSpeeds().setSpeeds("feedback", 
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond, 
            rController.calculate(
              robotBase.getLocalization().getFieldPose().getRotation().getDegrees(),
              shotTracker.getAngleToTarget().getDegrees())
            );
        }
      }
    );

    NamedCommands.registerCommand("Orient", new InstantCommand(() -> robotBase.getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(-2.3631872270622845)))));

    chooser = Autos.AUTOS.createChooser(AUTOS.Left);
    SmartDashboard.putData(chooser);

  }

  private void configureBindings() 
  {
    s.getSensor("Intake").getTrigger().onTrue(() -> s.setGoalState(SuperStructureStates.Home));
    driverController.leftTrigger.tiggerAt(0.5).onTrue(() -> {s.setGoalState(SuperStructureStates.Aim); isAiming = true;});
    driverController.leftTrigger.tiggerAt(0.5).onFalse(() -> {s.setGoalState(SuperStructureStates.Home); isAiming = false;});
    driverController.rightTrigger.tiggerAt(0.5).onTrue(() -> s.setGoalState(SuperStructureStates.Intaking));

    driverController.leftBumper.whileTrue(new AutoAling(climbTracker));
    

    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> {robotBase.getLocalization().resetFieldPose(0,0, 0); robotBase.getLocalization().resetRelativePose(0,0, 0);});
  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
