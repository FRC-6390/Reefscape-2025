// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;
import frc.robot.commands.auto.AlgaeAlign;
import frc.robot.commands.auto.BasicAlign;
import frc.robot.commands.auto.TagAlign;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.utils.ReefScoringPos.ReefPole;
import frc.robot.subsystems.superstructure.EndEffector;

public class RobotContainer {
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build();//.shuffleboard("Arm", SendableLevel.COMP);
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build();//.shuffleboard("Wrist", SendableLevel.COMP);
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.CORAL_ROLLERS.build();//.shuffleboard("Rollers", SendableLevel.COMP);;
  public final StatefulMechanism<RollerState> algaeRollers = Constants.EndEffector.ALGAE_ROLLERS.build();//.shuffleboard("Algae Rollers", SendableLevel.COMP);;

  public final Elevator elevator = new Elevator();
  public final EndEffector endEffector = new EndEffector(arm, wrist, rollers, algaeRollers).setAutoEndScoring(true);
  public Superstructure superstructure = new Superstructure(elevator, endEffector);
  public CANdleSubsystem candle = new CANdleSubsystem(robotBase);
  public static SuperstructureState selectedState = SuperstructureState.L4;

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(0.15)
                                                              .setLeftSlewrate(1)
                                                              ;
                                                              

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 
  public SendableChooser<Command> chooser;
  
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


    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    
    NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.HomePID));
    NamedCommands.registerCommand("Intake", Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()));

    NamedCommands.registerCommand("L4", superstructure.setState(SuperstructureState.L4));
    NamedCommands.registerCommand("L3", superstructure.setState(SuperstructureState.L3));
    NamedCommands.registerCommand("L2", superstructure.setState(SuperstructureState.L2));
    NamedCommands.registerCommand("L1", superstructure.setState(SuperstructureState.L1));

    NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    NamedCommands.registerCommand("WaitForElevator",superstructure.WaitForElevator());
    NamedCommands.registerCommand("WaitForEffector",superstructure.WaitForL4());
    NamedCommands.registerCommand("WaitForEjector", superstructure.WaitForEjector());

    NamedCommands.registerCommand("AlignRight", new TagAlign(robotBase, "limelight-right", superstructure, () -> selectedState));// new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    NamedCommands.registerCommand("AlignLeft", new TagAlign(robotBase, "limelight-left", superstructure, () -> selectedState)); //new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22),ReefPole.NONE, candle));
    NamedCommands.registerCommand("AlignRightK", new TagAlign(robotBase, "limelight-right", superstructure, () -> selectedState));//  new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
    NamedCommands.registerCommand("AlignLeftK", new TagAlign(robotBase, "limelight-left", superstructure, () -> selectedState)); //new InstantCommand(() -> candle.setRGB(0, 0, 255)), Units.inchesToMeters(22), ReefPole.K,candle));
  
    NamedCommands.registerCommand("TagAlignLeft", new TagAlign(robotBase, "limelight-left", superstructure, () -> selectedState));
    NamedCommands.registerCommand("TagAlignRight", new TagAlign(robotBase, "limelight-right", superstructure, () -> selectedState));
  
    NamedCommands.registerCommand("BasicAlignLeft", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase,"limelight-left")));
    NamedCommands.registerCommand("BasicAlignRight", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-right")));
    NamedCommands.registerCommand("BasicAlignLeftK", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-left", ReefPole.K)));
    NamedCommands.registerCommand("BasicAlignRightK", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-right", ReefPole.K)));
    NamedCommands.registerCommand("BasicAlignLeftA", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-left", ReefPole.A)));
    NamedCommands.registerCommand("BasicAlignRightA", Commands.sequence(superstructure.setState(SuperstructureState.Align),new BasicAlign(robotBase, "limelight-right", ReefPole.A)));
    NamedCommands.registerCommand("DisableLocal", 
    new  InstantCommand(() ->
    {
      robotBase.getVision().getLimelight("limelight-left").setUseForLocalization(false);
      robotBase.getVision().getLimelight("limelight-right").setUseForLocalization(false);
      robotBase.getVision().getPhotonVision("Tag").setUseForLocalization(false); 
      robotBase.getVision().getPhotonVision("TagFront").setUseForLocalization(false);
    }));
    NamedCommands.registerCommand("EnableLocal", 
    new InstantCommand(() ->
    {
      robotBase.getVision().getLimelight("limelight-left").setUseForLocalization(true); 
      robotBase.getVision().getLimelight("limelight-right").setUseForLocalization(true);
      robotBase.getVision().getPhotonVision("Tag").setUseForLocalization(true); 
      robotBase.getVision().getPhotonVision("TagFront").setUseForLocalization(true);
    }));


    chooser = Autos.AUTOS.createChooser(AUTOS.RIGHTSIDE);
    SmartDashboard.putData(chooser);
  }

  public boolean isIntaking()
  {
    return superstructure.stateMachine.getGoalState().equals(SuperstructureState.Intaking);
  }

  private void configureBindings() 
  {
    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    //RESET ODOMETRY
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0, 0));

    driverController.leftBumper.onTrue(
      Commands.either(
        Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()),
        superstructure.setState(SuperstructureState.HomePID),
        () -> !isIntaking()));
    driverController.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    // driverController.leftTrigger.tiggerAt(0.5).whileTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.Home));
    // driverController.rightTrigger.tiggerAt(0.5).whileTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.Home));
    // driverController.pov.right.whileTrue(Commands.sequence(superstructure.setState(SuperstructureState.Align),new TagAlign(robotBase, "limelight-left", superstructure, () -> selectedState))).onFalse(superstructure.setState(SuperstructureState.HomePID));
    // driverController.leftTrigger.tiggerAt(0.5).whileTrue(Commands.sequence(superstructure.setState(SuperstructureState.Align),new TagAlign(robotBase, "limelight-right", superstructure, () -> selectedState))).onFalse(superstructure.setState(SuperstructureState.HomePID));
    // driverController.rightTrigger.tiggerAt(0.5).whileTrue(Commands.sequence(superstructure.setState(SuperstructureState.Align),new TagAlign(robotBase, "limelight-left", superstructure, () -> selectedState))).onFalse(superstructure.setState(SuperstructureState.HomePID));

    driverController.pov.down.onTrue(superstructure.setState(SuperstructureState.HomePID)).after(1).onTrue(superstructure.setState(SuperstructureState.Home));

    // driverController.a.onTrue(() -> selectedState = SuperstructureState.L1);//after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.b.onTrue(() -> selectedState = SuperstructureState.L2);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.x.onTrue(() -> selectedState = SuperstructureState.L3);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.y.onTrue(() -> selectedState = SuperstructureState.L4);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);

    //---------------------------------------------------------DRIVER 2--------------------------------------------------//

    driverController.a.onTrue(superstructure.setState(SuperstructureState.L1)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.b.onTrue(superstructure.setState(SuperstructureState.L2)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.x.onTrue(superstructure.setState(SuperstructureState.L3)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController.y.onTrue(superstructure.setState(SuperstructureState.L4)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
  
    // driverController2.leftTrigger.tiggerAt(0.5).whileTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.Home));
    // driverController2.rightTrigger.tiggerAt(0.5).whileTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.Home));
    
    driverController2.start.whileTrue(superstructure.setState(SuperstructureState.AlgaeSpit)).after(1).onTrue(superstructure.setState(SuperstructureState.ScoreAlgae));
    driverController2.pov.up.onTrue(() -> elevator.nudge(1)).after(1).onTrue(() -> elevator.resetNudge());
    driverController2.pov.down.onTrue(() -> elevator.nudge(-1)).after(1).onTrue(() -> elevator.resetNudge());
    driverController2.pov.right.onTrue(() -> arm.setNudge(arm.getNudge() + 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.pov.left.onTrue(() -> arm.setNudge(arm.getNudge() - 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.rightBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() + 5)).after(1).onTrue(() -> wrist.setNudge(0));
    driverController2.leftBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() - 5)).after(1).onTrue(() -> wrist.setNudge(0));
    driverController.rightTrigger.tiggerAt(0.5).whileTrue(new V2(robotBase, "limelight-left")).onFalse(() -> robotBase.getDrivetrain().getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds()));
  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
