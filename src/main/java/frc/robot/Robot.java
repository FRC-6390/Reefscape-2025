// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.RollerState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.EndEffector;

public class Robot extends RobotBase<SwerveDrivetrain> {

  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build();//.shuffleboard("Arm", SendableLevel.DEBUG);
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build();//.shuffleboard("Wrist", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.CORAL_ROLLERS.build();//.shuffleboard("Rollers", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> algaeRollers = Constants.EndEffector.ALGAE_ROLLERS.build();//.shuffleboard("Algae Rollers", SendableLevel.COMP);;
  public final StatefulElevatorMechanism<ElevatorState> elevator = Constants.Elevator.ELEVATOR_CONFIG.build().shuffleboard("Elevator", SendableLevel.DEBUG);

  public final EndEffector endEffector = new EndEffector(arm, wrist, rollers, algaeRollers).setAutoEndScoring(false);
  public Superstructure superstructure = new Superstructure(elevator, endEffector);
  public CANdleSubsystem candle = new CANdleSubsystem(this);
  
  private final EnhancedXboxController driverController = new EnhancedXboxController(0).setLeftInverted(true).setRightInverted(true).setSticksDeadzone(0.15).setLeftSlewrate(5);
  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 

  public static SuperstructureState selectedState = SuperstructureState.L4;
  public V2 alignRight = new V2(this, "limelight-left", true, superstructure, () -> selectedState);
  public V2 alginLeft = new V2(this, "limelight-right", false, superstructure, () -> selectedState);

  PowerDistribution pdh;

  public Robot() {  
    super(Constants.DriveTrain.ROBOT_BASE);
    shuffleboard(SendableLevel.DEBUG);
    registerMechanism(arm, wrist, rollers, algaeRollers, elevator);

    getDrivetrain().setDriveCommand(driverController);
    getLocalization().setSuppressUpdates(false);

    arm.setPidEnabled(true);
    wrist.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setFeedforwardEnabled(false);

    pdh = new PowerDistribution(14, ModuleType.kRev);

    configureDriverController(driverController);
    configureOperatorController(driverController2);
  }

  @Override
  protected void onRobotInit() {
     pdh.clearStickyFaults();
     getLocalization().resetRelativePose(0, 0, 0);
  }

  @Override
  protected void onDisabledInit() {
      superstructure.setSuper(SuperstructureState.Home);
  }

  @Override
  public void onAutonomousExit() {
    Rotation2d offset = Rotation2d.fromDegrees(DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : 180);
    getIMU().setVirtualAxis("driver", getIMU().getVirtualAxis("field").minus(offset));
  }

  public void configureDriverController(EnhancedXboxController controller)
  {

    controller.start.onTrue(() -> getDrivetrain().getIMU().setYaw(0))
                    .after(2).onTrue(() -> {getLocalization().resetFieldPose(0,0, 0); getLocalization().resetRelativePose(0,0, 0);});

    controller.leftBumper.onTrue(
      Commands.either(
        Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()),
        superstructure.setState(SuperstructureState.HomePID),
        () -> !superstructure.stateMachine.getGoalState().equals(SuperstructureState.Intaking)));

    controller.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    controller.pov.left.whileTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.Home));
    controller.pov.right.whileTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.Home));
   
    controller.rightTrigger.tiggerAt(0.5)
    .onTrue(()-> CommandScheduler.getInstance().schedule(alignRight))
    .onFalse(() -> 
    {
      CommandScheduler.getInstance().cancel(alignRight); 
      // CommandScheduler.getInstance().schedule(superstructure.setState(SuperstructureState.Score));
    }
    );
    controller.leftTrigger.tiggerAt(0.5).onTrue(()-> CommandScheduler.getInstance().schedule(alginLeft)).onFalse(() -> {CommandScheduler.getInstance().cancel(alginLeft);});
    controller.pov.down.onTrue(superstructure.setState(SuperstructureState.HomePID)).after(1).onTrue(superstructure.setState(SuperstructureState.Home));

    controller.a.onTrue(() -> selectedState = SuperstructureState.L1);
    controller.b.onTrue(() -> selectedState = SuperstructureState.L2);
    controller.x.onTrue(() -> selectedState = SuperstructureState.L3);
    controller.y.onTrue(() -> selectedState = SuperstructureState.L4);

  
  }

  public void configureOperatorController(EnhancedXboxController controller)
  {
      controller.a.onTrue(superstructure.setState(SuperstructureState.L1)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
      controller.b.onTrue(superstructure.setState(SuperstructureState.L2)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
      controller.x.onTrue(superstructure.setState(SuperstructureState.L3)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
      controller.y.onTrue(superstructure.setState(SuperstructureState.L4)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    
      controller.start.whileTrue(superstructure.setState(SuperstructureState.AlgaeSpit)).after(1).onTrue(superstructure.setState(SuperstructureState.ScoreAlgae));
      controller.pov.right.onTrue(() -> arm.setNudge(arm.getNudge() + 5)).after(1).onTrue(() -> arm.setNudge(0));
      controller.pov.left.onTrue(() -> arm.setNudge(arm.getNudge() - 5)).after(1).onTrue(() -> arm.setNudge(0));
      controller.rightBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() + 5)).after(1).onTrue(() -> wrist.setNudge(0));
      controller.leftBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() - 5)).after(1).onTrue(() -> wrist.setNudge(0));
      controller.setLeftInverted(true).setRightInverted(true).setSticksDeadzone(0.15).setLeftSlewrate(5);
  }

  public void configureAutos(RobotAuto auto){

    auto.registerNamedCommand("WaitForTag", Commands.waitUntil(() -> getVision().getCameras().values().stream().anyMatch(LocalizationCamera::hasValidTarget)));
    
    auto.registerNamedCommand("Home", superstructure.setState(SuperstructureState.HomePID));
    auto.registerNamedCommand("OrientLeftSide", () -> getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(-2.3631872270622845))));

    auto.registerNamedCommand("Intake", Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()));

    auto.registerNamedCommand("L4", superstructure.setState(SuperstructureState.L4));
    auto.registerNamedCommand("L3", superstructure.setState(SuperstructureState.L3));
    auto.registerNamedCommand("L2", superstructure.setState(SuperstructureState.L2));
    auto.registerNamedCommand("L1", superstructure.setState(SuperstructureState.L1));

    auto.registerNamedCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    auto.registerNamedCommand("WaitForElevator",superstructure.WaitForElevator());
    auto.registerNamedCommand("WaitForEffector",superstructure.WaitForL4());
    auto.registerNamedCommand("WaitForEjector", superstructure.WaitForEjector());

    auto.registerNamedCommand("AlignRight", alignRight);
    auto.registerNamedCommand("AlignLeft", alginLeft);
    auto.registerNamedCommand("DisableLocal", () -> getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(false)));
    auto.registerNamedCommand("EnableLocal", () -> getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(true)));

    auto.registerPathPlannerAuto("Left", "CompLeftSide");
    auto.registerPathPlannerAuto("Right", "CompMidSide");
    auto.registerPathPlannerAuto("Mid", "CompMidSide");
  }

}
