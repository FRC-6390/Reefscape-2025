// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureMechanism;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Elevator.ElevatorState;
import frc.robot.Constants.Superstructure.SuperstructureState;
import frc.robot.Constants.Superstructure.EndEffectorState;
import frc.robot.Constants.Superstructure.EndEffectorTuple;
import frc.robot.Constants.Superstructure.SuperstructureTuple;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.superstructure.CANdleSubsystem;

public class Robot extends RobotCore<SwerveDrivetrain> {

  private final SuperstructureMechanism<SuperstructureState, SuperstructureTuple> superstructure;
  private final SuperstructureMechanism<EndEffectorState, EndEffectorTuple> endEffector;
  public final StatefulArmMechanism<Constants.EndEffector.ArmState> arm;//.shuffleboard("Arm", SendableLevel.DEBUG);
  public final StatefulArmMechanism<Constants.EndEffector.WristState> wrist;//.shuffleboard("Wrist", SendableLevel.DEBUG);
  public final StatefulMechanism<Constants.EndEffector.RollerState> rollers;//.shuffleboard("Rollers", SendableLevel.DEBUG);
  public final StatefulMechanism<Constants.EndEffector.RollerState> algaeRollers;//.shuffleboard("Algae Rollers", SendableLevel.COMP);;
  public final StatefulElevatorMechanism<ElevatorState> elevator;

  public CANdleSubsystem candle = new CANdleSubsystem(this);
  
  private final EnhancedXboxController driverController = new EnhancedXboxController(0).setLeftInverted(true).setRightInverted(true).setSticksDeadzone(0.15).setLeftSlewrate(5);
  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 

  public static SuperstructureState selectedState = SuperstructureState.L4;
  public V2 alignRight;
  public V2 alginLeft;

  PowerDistribution pdh;

  public Robot() {  
    super(Constants.DriveTrain.ROBOT_BASE);

    superstructure = Constants.Superstructure.SUPERSTRUCTURE_CONFIG.build();
    endEffector = superstructure.getMechanisms().superstructure(SuperstructureTuple::endEffector);
    arm = endEffector.getMechanisms().arm(EndEffectorTuple::joint1state);
    wrist = endEffector.getMechanisms().arm(EndEffectorTuple::joint2state);
    rollers = endEffector.getMechanisms().generic(EndEffectorTuple::coralRollerState);
    algaeRollers = endEffector.getMechanisms().generic(EndEffectorTuple::algaeRollerState);
    elevator = superstructure.getMechanisms().elevator(SuperstructureTuple::elevator);

    shuffleboard(SendableLevel.DEBUG);
    registerMechanism(superstructure);

    getDrivetrain().setDriveCommand(driverController);
    getLocalization().setSuppressUpdates(false);

    arm.setPidEnabled(true);
    wrist.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setFeedforwardEnabled(false);

    pdh = new PowerDistribution(14, ModuleType.kRev);

    configureDriverController(driverController);
    configureOperatorController(driverController2);

    alignRight = new V2(this, "limelight-left", true, superstructure, () -> selectedState);
    alginLeft = new V2(this, "limelight-right", false, superstructure, () -> selectedState);
  }

  @Override
  protected void onRobotInit() {
     pdh.clearStickyFaults();
     getLocalization().resetRelativePose(0, 0, 0);
  }

  @Override
  protected void onDisabledInit() {
      setSuper(SuperstructureState.Home);
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
        Commands.either(setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.inputSupplier("hasPiece").getAsBoolean()),
        setState(SuperstructureState.HomePID),
        () -> !superstructure.getStateMachine().getGoalState().equals(SuperstructureState.Intaking)));

    controller.rightBumper.onTrue(setState(SuperstructureState.Score));

    controller.pov.left.whileTrue(setState(SuperstructureState.AlgaeLow)).onFalse(setState(SuperstructureState.Home));
    controller.pov.right.whileTrue(setState(SuperstructureState.AlgaeHigh)).onFalse(setState(SuperstructureState.Home));
   
    controller.rightTrigger.tiggerAt(0.5)
    .onTrue(()-> CommandScheduler.getInstance().schedule(alignRight))
    .onFalse(() -> 
    {
      CommandScheduler.getInstance().cancel(alignRight); 
      // CommandScheduler.getInstance().schedule(superstructure.setState(SuperstructureState.Score));
    }
    );
    controller.leftTrigger.tiggerAt(0.5).onTrue(()-> CommandScheduler.getInstance().schedule(alginLeft)).onFalse(() -> {CommandScheduler.getInstance().cancel(alginLeft);});
    controller.pov.down.onTrue(setState(SuperstructureState.HomePID)).after(1).onTrue(setState(SuperstructureState.Home));

    controller.a.onTrue(() -> selectedState = SuperstructureState.L1);
    controller.b.onTrue(() -> selectedState = SuperstructureState.L2);
    controller.x.onTrue(() -> selectedState = SuperstructureState.L3);
    controller.y.onTrue(() -> selectedState = SuperstructureState.L4);

  
  }

  public void configureOperatorController(EnhancedXboxController controller)
  {
      controller.a.onTrue(setState(SuperstructureState.L1)).after(0.75).onTrue(setState(SuperstructureState.Score));
      controller.b.onTrue(setState(SuperstructureState.L2)).after(0.75).onTrue(setState(SuperstructureState.Score));
      controller.x.onTrue(setState(SuperstructureState.L3)).after(0.75).onTrue(setState(SuperstructureState.Score));
      controller.y.onTrue(setState(SuperstructureState.L4)).after(0.75).onTrue(setState(SuperstructureState.Score));
    
      controller.start.whileTrue(setState(SuperstructureState.AlgaeSpit)).after(1).onTrue(setState(SuperstructureState.ScoreAlgae));
    controller.pov.right.onTrue(() -> arm.setNudge(arm.getNudge() + 5)).after(1).onTrue(() -> arm.setNudge(0));
    controller.pov.left.onTrue(() -> arm.setNudge(arm.getNudge() - 5)).after(1).onTrue(() -> arm.setNudge(0));
    controller.rightBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() + 5)).after(1).onTrue(() -> wrist.setNudge(0));
    controller.leftBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() - 5)).after(1).onTrue(() -> wrist.setNudge(0));
    controller.setLeftInverted(true).setRightInverted(true).setSticksDeadzone(0.15).setLeftSlewrate(5);
  }

  public void configureAutos(RobotAuto auto){

    auto.registerNamedCommand("WaitForTag", Commands.waitUntil(() -> getVision().getCameras().values().stream().anyMatch(LocalizationCamera::hasValidTarget)));
    
    auto.registerNamedCommand("Home", setState(SuperstructureState.HomePID));
    auto.registerNamedCommand("OrientLeftSide", () -> getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(-2.3631872270622845))));

    auto.registerNamedCommand("Intake", Commands.either(setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.inputSupplier("hasPiece").getAsBoolean()));

    auto.registerNamedCommand("L4", setState(SuperstructureState.L4));
    auto.registerNamedCommand("L3", setState(SuperstructureState.L3));
    auto.registerNamedCommand("L2", setState(SuperstructureState.L2));
    auto.registerNamedCommand("L1", setState(SuperstructureState.L1));

    auto.registerNamedCommand("StartEject", setState(SuperstructureState.Score));
    auto.registerNamedCommand("WaitForElevator",superstructure.getStateMachine().waitUntilAtGoal());
    auto.registerNamedCommand("WaitForEffector",endEffector.getStateMachine().waitUntil(EndEffectorState.L4));
    auto.registerNamedCommand("WaitForEjector", new edu.wpi.first.wpilibj2.command.WaitUntilCommand(endEffector.inputSupplier("hasPiece")));

    auto.registerNamedCommand("AlignRight", alignRight);
    auto.registerNamedCommand("AlignLeft", alginLeft);
    auto.registerNamedCommand("DisableLocal", () -> getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(false)));
    auto.registerNamedCommand("EnableLocal", () -> getVision().getCameras().values().forEach(camera -> camera.setUseForLocalization(true)));

    auto.registerPathPlannerAuto("Left", "CompLeftSide");
    auto.registerPathPlannerAuto("Right", "CompMidSide");
    auto.registerPathPlannerAuto("Mid", "CompMidSide");
  }

  private InstantCommand setState(SuperstructureState state) {
    return new InstantCommand(() -> superstructure.getStateMachine().queueState(state));
  }

  private void setSuper(SuperstructureState state) {
    superstructure.getStateMachine().queueState(state);
  }

}
