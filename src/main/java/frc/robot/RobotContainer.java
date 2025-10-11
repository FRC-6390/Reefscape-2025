// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.EndEffector;

public class RobotContainer {
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard(SendableLevel.DEBUG);
 
  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build();//.shuffleboard("Arm", SendableLevel.DEBUG);
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build();//.shuffleboard("Wrist", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.CORAL_ROLLERS.build();//.shuffleboard("Rollers", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> algaeRollers = Constants.EndEffector.ALGAE_ROLLERS.build();//.shuffleboard("Algae Rollers", SendableLevel.COMP);;
  public final StatefulElevatorMechanism<ElevatorState> elevator = Constants.Elevator.ELEVATOR_CONFIG.build() ;//.shuffleboard("Elevator", SendableLevel.DEBUG);
  // public SuperStructureTest s = SuperstructureBuilder.builder().addArms(arm, wrist).addMotors(rollers, algaeRollers).build();
  public BooleanSupplier hasTarget;
  // public final Elevator elevator = new Elevator();
  public final EndEffector endEffector = new EndEffector(arm, wrist, rollers, algaeRollers).setAutoEndScoring(false);
  public Superstructure superstructure = new Superstructure(elevator, endEffector);
  public CANdleSubsystem candle = new CANdleSubsystem(robotBase);
  public static SuperstructureState selectedState = SuperstructureState.L4;
  public V2 alignRight = new V2(robotBase, "limelight-left", true, superstructure, () -> selectedState);
  public V2 alginLeft = new V2(robotBase, "limelight-right", false, superstructure, () -> selectedState);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(0.15)
                                                              .setLeftSlewrate(5);

                                                        
                                                              

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 
  public SendableChooser<Command> chooser;

  
  
  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    robotBase.registerMechanism(arm, algaeRollers, wrist, rollers, elevator);
    robotBase.getLocalization().setSuppressUpdates(false);
    arm.setPidEnabled(true);
    wrist.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setFeedforwardEnabled(false);


    elevator.shuffleboard("Elevator", SendableLevel.DEBUG);

  
    hasTarget = () -> {
      if (robotBase.getVision() == null) {
        return false;
      }
      return robotBase.getVision().getCameras().values().stream().anyMatch(LocalizationCamera::hasValidTarget);
    };
    NamedCommands.registerCommand("WaitForTag", Commands.waitUntil(hasTarget));
    
    NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.HomePID));
    NamedCommands.registerCommand("OrientLeftSide", new InstantCommand(() -> robotBase.getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(-2.3631872270622845)))));

    NamedCommands.registerCommand("Intake", Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()));

    NamedCommands.registerCommand("L4", superstructure.setState(SuperstructureState.L4));
    NamedCommands.registerCommand("L3", superstructure.setState(SuperstructureState.L3));
    NamedCommands.registerCommand("L2", superstructure.setState(SuperstructureState.L2));
    NamedCommands.registerCommand("L1", superstructure.setState(SuperstructureState.L1));

    NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    NamedCommands.registerCommand("WaitForElevator",superstructure.WaitForElevator());
    NamedCommands.registerCommand("WaitForEffector",superstructure.WaitForL4());
    NamedCommands.registerCommand("WaitForEjector", superstructure.WaitForEjector());

    NamedCommands.registerCommand("AlignRight", alignRight);
    NamedCommands.registerCommand("AlignLeft", alginLeft);
    NamedCommands.registerCommand("DisableLocal",
    new  InstantCommand(() -> {
      if (robotBase.getVision() == null) {
        return;
      }
      robotBase.getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(false));
    }));
    NamedCommands.registerCommand("EnableLocal",
    new InstantCommand(() -> {
      if (robotBase.getVision() == null) {
        return;
      }
      robotBase.getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(true));
    }));


    chooser = Autos.AUTOS.createChooser(AUTOS.Left);
    SmartDashboard.putData(chooser);
  }

  public boolean isIntaking()
  {
    return superstructure.stateMachine.getGoalState().equals(SuperstructureState.Intaking);
  }

  private void configureBindings() 
  {

    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> {robotBase.getLocalization().resetFieldPose(0,0, 0); robotBase.getLocalization().resetRelativePose(0,0, 0);});

    driverController.leftBumper.onTrue(
      Commands.either(
        Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()),
        superstructure.setState(SuperstructureState.HomePID),
        () -> !isIntaking()));

    driverController.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    driverController.pov.left.whileTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.Home));
    driverController.pov.right.whileTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.Home));
   
    driverController.rightTrigger.tiggerAt(0.5)
    .onTrue(()-> CommandScheduler.getInstance().schedule(alignRight))
    .onFalse(() -> 
    {
      CommandScheduler.getInstance().cancel(alignRight); 
      // CommandScheduler.getInstance().schedule(superstructure.setState(SuperstructureState.Score));
    }
    );
    driverController.leftTrigger.tiggerAt(0.5).onTrue(()-> CommandScheduler.getInstance().schedule(alginLeft)).onFalse(() -> {CommandScheduler.getInstance().cancel(alginLeft);});
    driverController.pov.down.onTrue(superstructure.setState(SuperstructureState.HomePID)).after(1).onTrue(superstructure.setState(SuperstructureState.Home));

    driverController.a.onTrue(() -> selectedState = SuperstructureState.L1);//after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    driverController.b.onTrue(() -> selectedState = SuperstructureState.L2);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    driverController.x.onTrue(() -> selectedState = SuperstructureState.L3);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    driverController.y.onTrue(() -> selectedState = SuperstructureState.L4);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);

    //---------------------------------------------------------DRIVER 2--------------------------------------------------//

    driverController2.a.onTrue(superstructure.setState(SuperstructureState.L1)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController2.b.onTrue(superstructure.setState(SuperstructureState.L2)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController2.x.onTrue(superstructure.setState(SuperstructureState.L3)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    driverController2.y.onTrue(superstructure.setState(SuperstructureState.L4)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
  
    driverController2.start.whileTrue(superstructure.setState(SuperstructureState.AlgaeSpit)).after(1).onTrue(superstructure.setState(SuperstructureState.ScoreAlgae));
    // driverController2.start.onTrue(() -> s.requestState2(S.Intaking.getSetpoint()));
    // driverController2.pov.up.onTrue(() -> elevator.nudge(1)).after(1).onTrue(() -> elevator.resetNudge());
    // driverController2.pov.down.onTrue(() -> elevator.nudge(-1)).after(1).onTrue(() -> elevator.resetNudge());
    driverController2.pov.right.onTrue(() -> arm.setNudge(arm.getNudge() + 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.pov.left.onTrue(() -> arm.setNudge(arm.getNudge() - 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.rightBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() + 5)).after(1).onTrue(() -> wrist.setNudge(0));
    driverController2.leftBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() - 5)).after(1).onTrue(() -> wrist.setNudge(0));

    

  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
