// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import au.grapplerobotics.LaserCan;
import frc.robot.commands.auto.TagAlign;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.Climber.ClimberState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;

import frc.robot.commands.auto.AtState;
import frc.robot.commands.auto.AutoPickup;
import frc.robot.commands.auto.BasicAlign;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffectorV2.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefLevel;
import frc.robot.utils.ReefScoringPos.ReefPole;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffectorV2;

public class RobotContainer {


  public final LaserCan lasLeft = new LaserCan(1);
  public final LaserCan lasRight = new LaserCan(0);
  
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  // public final StatefulMechanism<ClimberState> climberTest = Constants.Climber.CLIMBER_CONFIG.build().shuffleboard("Climber Test");
  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build().shuffleboard("Arm");
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build().shuffleboard("Wrist");
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.ROLLER_CONFIG.build().shuffleboard("Rollers");

  double speed = 0.5;
  double current = 40;

  public Elevator elevator = new Elevator();
  // public Climber climber = new Climber();
  public EndEffectorV2 endEffector = new EndEffectorV2(arm, wrist, rollers);
  // public Superstructure superstructure = new Superstructure(elevator, endEffector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem(robotBase);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(2);

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE);
                   

  // public Elevate elevate = new Elevate(ElevatorState.Home, lasLeft, lasRight, superstructure, robotBase, elevator);
  public SendableChooser<Command> chooser;
  public RobotContainer() 
  {
    configureBindings();
    arm.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setPidEnabled(true);
    wrist.setFeedforwardEnabled(false);
    rollers.setPidEnabled(true);
    rollers.setFeedforwardEnabled(false);
    Shuffleboard.getTab("Elevator").addDouble("Speed", () -> speed);
    Shuffleboard.getTab("Elevator").addDouble("Current", () -> current);

    robotBase.getDrivetrain().setDriveCommand(driverController);
   
    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    // endEffector.shuffleboard("Effector");
    // climber.shuffleboard("climber");
    
    // NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.Home));
    // NamedCommands.registerCommand("ManualL4", superstructure.setState(SuperstructureState.L4));
    // NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    // NamedCommands.registerCommand("WaitForElevator",Commands.race( new AtState(superstructure), new WaitCommand(3)));
    // NamedCommands.registerCommand("WaitForEjector", Commands.race( new AtStateEjector(endEffector), new WaitCommand(3)));
    NamedCommands.registerCommand("AlignRight", new TagAlign(robotBase, "limelight-right", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    NamedCommands.registerCommand("AlignLeft", new TagAlign(robotBase, "limelight-left", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    NamedCommands.registerCommand("AlignRightK", new TagAlign(robotBase, "limelight-right",  new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
    NamedCommands.registerCommand("AlignLeftK", new TagAlign(robotBase, "limelight-left", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
   
    NamedCommands.registerCommand("BasicAlignLeft", new BasicAlign(robotBase, "limelight-left"));
    NamedCommands.registerCommand("BasicAlignRight", new BasicAlign(robotBase, "limelight-right"));
    NamedCommands.registerCommand("BasicAlignLeftK", new BasicAlign(robotBase, "limelight-left", ReefPole.K));
    NamedCommands.registerCommand("BasicAlignRightK", new BasicAlign(robotBase, "limelight-right", ReefPole.K));

    NamedCommands.registerCommand("Home", new InstantCommand(() -> candle.setRGB(0, 0, 0)));
    NamedCommands.registerCommand("Score", new InstantCommand(() -> candle.setRGB(0, 255, 0)));
    
    // climberTest.setPidEnabled(false);
    // climberTest.setFeedforwardEnabled(false);

    chooser = Autos.AUTOS.createChooser(AUTOS.LEFTSIDE);
    SmartDashboard.putData(chooser);
  }

  private void configureBindings() 
  {

    //TODO
        //  TEST RETURN TO SCORE
        //  TEST STRAFE

        //AUTO TESTING
          //  REDO OFFSETS, TEST AUTO ON ROBOT CART, CHECK WITH TAG IF LOCALIZATION IS STILL WORKING AFTER
          // IF NOT
              //ALTERNATIVE 1
                //  DURING MATCH, TEST STRAIGHT LINE IN CORRECT START CONFIGURATION
                //      IF THAT WORKS, NEXT MATCH, START, ROTAT, THEN STRAIGHT LINE 
              //ALTERNATIVE 2
                //  TEST PASSIVE ALIGN
              //ALTERNATIVE 3
                // TEST STRAFE FOR SECONDS IN AUTO
          

    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    //RESET ODOMETRY
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0, 0));
    // driverController.leftBumper.whileTrue(() -> elevator.setMotors(speed)).onFalse(() -> elevator.setMotors(0));
    // driverController.rightBumper.whileTrue(() -> elevator.setMotors(-speed)).onFalse(() -> elevator.setMotors(0));
    // driverController.pov.up.onTrue(() -> speed += 0.1);
    // driverController.pov.down.onTrue(() -> speed -= 0.1);

    // driverController.pov.left.onTrue(() -> {speed += 0.5; elevator.setCurrentLimit(current);});
    // driverController.pov.right.onTrue(() ->  {speed -= 0.5; elevator.setCurrentLimit(current);});


    // driverController.leftBumper.whileTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.L1)).after(1.25).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));
    driverController.rightBumper.whileTrue(() -> {elevator.getStateMachine().setGoalState(ElevatorState.Home); endEffector.getStateMachine().setGoalState(EndEffectorState.Intaking);});

    // driverController.b.whileTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Intaking));

    // driverController.x.whileTrue(() -> wrist.getStateMachine().setGoalState(WristState.Intaking));
    // driverController.y.whileTrue(() -> wrist.getStateMachine().setGoalState(WristState.Scoring));
    
    // driverController.a.whileTrue(() -> arm.setMotors(speed)).onFalse(() -> arm.setMotors(0));
    // driverController.b.whileTrue(() -> arm.setMotors(-speed)).onFalse(() -> arm.setMotors(0));
    // driverController.x.whileTrue(() -> wrist.setMotors(speed)).onFalse(() -> wrist.setMotors(0));
    // driverController.y.whileTrue(() -> wrist.setMotors(-speed)).onFalse(() -> wrist.setMotors(0));
    driverController.a.onTrue(() -> {endEffector.getStateMachine().setGoalState(EndEffectorState.L1);elevator.getStateMachine().setGoalState(ElevatorState.L1);}).after(2).onTrue(() -> {endEffector.getStateMachine().setGoalState(EndEffectorState.Score);});//.after(1.25).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));;
    driverController.b.onTrue(() -> {endEffector.getStateMachine().setGoalState(EndEffectorState.L2);elevator.getStateMachine().setGoalState(ElevatorState.L2);}).after(2).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));//.after(1.25).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));;
    driverController.x.onTrue(() -> {endEffector.getStateMachine().setGoalState(EndEffectorState.L3);elevator.getStateMachine().setGoalState(ElevatorState.L3);}).after(2).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));//.after(1.25).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));;
    driverController.y.onTrue(() -> {endEffector.getStateMachine().setGoalState(EndEffectorState.L4);elevator.getStateMachine().setGoalState(ElevatorState.L4);}).after(2).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));//.after(1.25).onTrue(() -> endEffector.getStateMachine().setGoalState(EndEffectorState.Score));;

    // driverController.b.whileTrue(() -> {

    //   System.out.println("running");
    //   var cams = robotBase.getVision().getPhotonVisions();
    //   System.out.println("size: "+cams.size());

    //   for (PhotonVision cam : cams.values()) {

    //       System.out.println(cam.getLocalizationPose());

    //       System.out.println("Connected: " + cam.isConnected());
    //       System.out.println("unread: " + cam.getAllUnreadResults().size());

        //  endEffector.getStateMachine().setGoalState(EndEffectorState.L4);elevator.getStateMachine().setGoalState(ElevatorState.L4);}).after(2).onTrue(() -> elevator.getStateMachine().setGoalState(ElevatorState.Home)

    //       // if (visionEst.isPresent()) {
    //       //     return visionEst.get().estimatedPose.toPose2d();
    //       // }

    //       // return null;
    //   }
      
    // });
    //PASSIVE ALIGN 
    // driverController.rightStick.toggleOnTrue(new PassiveAlign(robotBase));

    // //AUTO ALIGN (RIGHT BUMPER)
    
    // //EJECT PIECE MANUALLY
    // driverController.rightBumper.whileTrue(superstructure.setState(SuperstructureState.Score));
    // driverController.leftBumper.whileTrue(superstructure.setState(SuperstructureState.Score));

    
    // //SCORING COMMANDS
    // driverController.a.onTrue(superstructure.setState(SuperstructureState.Home));
    // driverController.b.onTrue(superstructure.setState(SuperstructureState.L2));
    // driverController.x.onTrue(superstructure.setState(SuperstructureState.L3));
    // driverController.y.onTrue(superstructure.setState(SuperstructureState.L4));

    // //ALGAE REMOVAL SEQUENCE
    // driverController.rightTrigger.tiggerAt(0.5).onTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    // driverController.leftTrigger.tiggerAt(0.5).onTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    
    //STRAFING
    // driverController.pov.left.whileTrue(new TagAlign(robotBase, "limelight-left", new InstantCommand(() -> candle.setRGB(255, 255, 255)), Units.inchesToMeters(22), ReefPole.K, candle));
    // driverController.pov.right.whileTrue(new TagAlign(robotBase, "limelight-right", new InstantCommand(() -> candle.setRGB(255, 255, 255)), Units.inchesToMeters(22), candle));
    driverController.pov.up.whileTrue(new BasicAlign(robotBase, "limelight-left"));
    driverController.pov.down.whileTrue(new BasicAlign(robotBase, "limelight-right"));

    // driverController.pov.right.onTrue(() -> superstructure.setState(SuperstructureState.L2)).after(1).onTrue(() -> climber.setClimber(10));
    // driverController.pov.left.onTrue(() -> superstructure.setState(SuperstructureState.L2)).after(1).onTrue(() -> climber.setClimber(0));
    // driverController.pov.right.whileTrue(() -> climberTest.setMotors(1)).onFalse(() -> climberTest.setMotors(0.0));
    // driverController.pov.left.whileTrue(() -> climberTest.setMotors(-1)).onFalse(() -> climberTest.setMotors(0.0));
    
    // driverController.pov.down.onTrue(() -> climberTest.setEncoderPosition(0));
    // //----------------------------------------------------------DRIVER 2---------------------------------------------------------------//

    // //FLIP EJECTION
    // driverController2.leftBumper.onTrue(() -> {endEffector.getRollers().setFlip(true); endEffector.getRotator().setFlip(true);});
    // driverController2.rightBumper.onTrue(() -> {endEffector.getRollers().setFlip(false); endEffector.getRotator().setFlip(false);});

    // //END EFFECTOR OVERRIDE
    // driverController2.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    // //ELEVATOR OVERIDE
    // driverController2.pov.left.onTrue(superstructure.setElevator(ElevatorState.Home));
    // driverController2.pov.up.onTrue(() -> elevator.nudge(1));
    // driverController2.pov.down.onTrue(() -> elevator.nudge(-1));
    // driverController2.start.after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0,Rotation2d.fromDegrees(90))));  
    // driverController2.leftBumper.whileTrue(() -> {robotBase.getRobotSpeeds().setAutoSpeeds(new ChassisSpeeds(0,0,0)); robotBase.getRobotSpeeds().stopAutoSpeeds();});
  }

  public Command getAutonomousCommand() 
  {
    
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
