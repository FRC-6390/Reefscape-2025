// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import au.grapplerobotics.LaserCan;
import frc.robot.commands.auto.TagAlign;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.Climber.ClimberState;
import frc.robot.commands.auto.AtState;
import frc.robot.commands.auto.AtStateEjector;
import frc.robot.commands.auto.RotateTo;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector;

public class RobotContainer {


  public final LaserCan lasLeft = new LaserCan(1);
  public final LaserCan lasRight = new LaserCan(0);
  
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  // public final StatefulMechanism<ClimberState> climberTest = Constants.Climber.CLIMBER_CONFIG.build().shuffleboard("Climber Test");


  public Elevator elevator = new Elevator();
  // public Climber climber = new Climber();
  // public EndEffector endEffector = new EndEffector(robotBase).setAutoEndScoring(true);
  // public Superstructure superstructure = new Superstructure(elevator, endEffector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem();

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
    robotBase.getDrivetrain().setDriveCommand(driverController);
   
    // elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    // endEffector.shuffleboard("Effector");
    // climber.shuffleboard("climber");
    
    // NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.Home));
    // NamedCommands.registerCommand("ManualL4", superstructure.setState(SuperstructureState.L4));
    // NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    // NamedCommands.registerCommand("WaitForElevator",Commands.race( new AtState(superstructure), new WaitCommand(3)));
    // NamedCommands.registerCommand("WaitForEjector", Commands.race( new AtStateEjector(endEffector), new WaitCommand(3)));
    NamedCommands.registerCommand("RotateToRight",new RotateTo(robotBase,Rotation2d.fromRadians(-0.49333207719329186)));
    NamedCommands.registerCommand("RotateToLeft", new RotateTo(robotBase,Rotation2d.fromRadians(-2.575148734982150)));
    NamedCommands.registerCommand("RotateToMid", new RotateTo(robotBase,Rotation2d.fromRadians(-1.601756394242849)));
    NamedCommands.registerCommand("AlignRight", new TagAlign(robotBase, "limelight-right",Units.inchesToMeters(55)));
    NamedCommands.registerCommand("AlignLeft", new TagAlign(robotBase, "limelight-left",Units.inchesToMeters(55)));
    NamedCommands.registerCommand("AlignRightFar", new TagAlign(robotBase, "limelight-right", Units.inchesToMeters(0)));
    NamedCommands.registerCommand("AlignLeftFar", new TagAlign(robotBase, "limelight-left", Units.inchesToMeters(90)));

    
    // climberTest.setPidEnabled(false);
    // climberTest.setFeedforwardEnabled(false);

    chooser = Autos.AUTOS.createChooser(AUTOS.PRELOADLEFT);
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
    driverController.leftBumper.onTrue(() -> elevator.setMotors(0.5)).onFalse(() -> elevator.setMotors(0));
    driverController.rightBumper.onTrue(() -> elevator.setMotors(-0.5)).onFalse(() -> elevator.setMotors(0));

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
    driverController.pov.left.whileTrue(new TagAlign(robotBase, "limelight-left", Units.inchesToMeters(90)));
    driverController.pov.right.whileTrue(new TagAlign(robotBase, "limelight-right", Units.inchesToMeters(90)));

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
