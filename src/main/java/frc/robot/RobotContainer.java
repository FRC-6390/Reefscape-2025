// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.commands.auto.TagAlign;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;

import frc.robot.commands.auto.BasicAlign;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.utils.ReefScoringPos.ReefPole;
import frc.robot.subsystems.superstructure.EndEffectorV2;
import frc.robot.subsystems.superstructure.EndEffectorV2.EndEffectorState;

public class RobotContainer {
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create();//.shuffleboard();
  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build().shuffleboard("Arm");
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build().shuffleboard("Wrist");
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.ROLLER_CONFIG.build();//.shuffleboard("Rollers");

  public Elevator elevator = new Elevator();
  public EndEffectorV2 endEffector = new EndEffectorV2(arm, wrist, rollers).setAutoEndScoring(true);
  public Superstructure superstructure = new Superstructure(elevator, endEffector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem(robotBase);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(2);

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE);
                   
  public String selectedLimelight = "limelight-left";

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
    robotBase.getDrivetrain().setDriveCommand(driverController);
   
    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    
    NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.Home));
    NamedCommands.registerCommand("L4", superstructure.setState(SuperstructureState.L4));
    NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    NamedCommands.registerCommand("WaitForElevator",superstructure.WaitForElevator());
    NamedCommands.registerCommand("WaitForEffector",superstructure.WaitForL4());
    NamedCommands.registerCommand("WaitForEjector", superstructure.WaitForEjector());

    // NamedCommands.registerCommand("AlignRight", new TagAlign(robotBase, "limelight-right", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    // NamedCommands.registerCommand("AlignLeft", new TagAlign(robotBase, "limelight-left", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    // NamedCommands.registerCommand("AlignRightK", new TagAlign(robotBase, "limelight-right",  new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
    // NamedCommands.registerCommand("AlignLeftK", new TagAlign(robotBase, "limelight-left", new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
   
    NamedCommands.registerCommand("BasicAlignLeft", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase,"limelight-left")));
    NamedCommands.registerCommand("BasicAlignRight", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-right")));
    NamedCommands.registerCommand("BasicAlignLeftK", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-left", ReefPole.K)));
    NamedCommands.registerCommand("BasicAlignRightK", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-right", ReefPole.K)));
    

    chooser = Autos.AUTOS.createChooser(AUTOS.LEFTSIDE);
    SmartDashboard.putData(chooser);
  
  }

  public InstantCommand selectLimelight(String limleight)
  {
    return new InstantCommand(() -> selectedLimelight = limleight);
  }

  private void configureBindings() 
  {
    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    //RESET ODOMETRY
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0, 0));

    driverController.leftBumper.onTrue(superstructure.setState(SuperstructureState.Intaking));
    driverController.rightBumper.onTrue(superstructure.setState(SuperstructureState.Home));

    driverController.pov.down.whileTrue(() -> System.out.println(robotBase.getVision().getPhotonVision("Tag").isConnected()));


    driverController.pov.left.whileTrue(new BasicAlign(robotBase, "limelight-left"));
    driverController.pov.right.whileTrue(new BasicAlign(robotBase, "limelight-right"));

    driverController.a.onTrue(Commands.sequence(superstructure.setState(SuperstructureState.L1))).after(1.2).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.b.onTrue(Commands.sequence(superstructure.setState(SuperstructureState.L2))).after(1.2).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.x.onTrue(Commands.sequence(superstructure.setState(SuperstructureState.L3))).after(1.2).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.y.onTrue(Commands.sequence(superstructure.setState(SuperstructureState.L4))).after(1.2).onTrue(superstructure.setState(SuperstructureState.Score));
  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
