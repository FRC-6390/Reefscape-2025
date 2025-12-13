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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autos.AUTOS;
import frc.robot.Constants.Elevator.S;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.RollerState;
import frc.robot.commands.GoingBald.AlignCamera;
import frc.robot.commands.GoingBald.AutoAling;
import frc.robot.commands.GoingBald.GeneralAlign;
import frc.robot.commands.auto.AlgaeAlign;
import frc.robot.commands.auto.BasicAlign;
import frc.robot.commands.auto.TagAlign;
import frc.robot.commands.auto.V2;
// import frc.robot.commands.rookie.Aim;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.utils.Experimental.ActionableConstraint;
import frc.robot.utils.Experimental.Constraint;
import frc.robot.utils.Experimental.DigitalSensor;
import frc.robot.utils.Experimental.SuperStructureStates;
import frc.robot.utils.Experimental.SuperStructureTest;
import frc.robot.utils.Experimental.SuperstructureBuilder;
import frc.robot.utils.ReefScoringPos.ReefPole;
import frc.robot.subsystems.superstructure.EndEffector;

//TODO
//CHECK IF IT WORKS ON RED AND ANOTHER SIDE, CHECK IF AUTO WORKS, CHECK 

public class RobotContainer {
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
 
  public PIDController rController = new PIDController(0.11, 0, 0);
  public AlignCamera camLeft = new AlignCamera(robotBase.getVision().getLimelight("limelight-left"), -Units.inchesToMeters(0.5), -Units.inchesToMeters(9.25), 15, 0);
    public AlignCamera camRight = new AlignCamera(robotBase.getVision().getLimelight("limelight-right"), -Units.inchesToMeters(0.5), Units.inchesToMeters(9.25), -15, 0);


  public HolonomicDriveController controller = new HolonomicDriveController(
                                                          new PIDController(1, 0, 0), 
                                                          new PIDController(1, 0, 0),
                                                          new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));
  public final StatefulArmMechanism<ArmState> arm = Constants.EndEffector.ARM_CONFIG.build().shuffleboard("Arm", SendableLevel.DEBUG);
  public final StatefulArmMechanism<WristState> wrist = Constants.EndEffector.WRIST_CONFIG.build().shuffleboard("Wrist", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> rollers = Constants.EndEffector.CORAL_ROLLERS.build();///.shuffleboard("Rollers", SendableLevel.DEBUG);
  public final StatefulMechanism<RollerState> algaeRollers = Constants.EndEffector.ALGAE_ROLLERS.build();//.shuffleboard("Algae Rollers", SendableLevel.COMP);;

  public SuperStructureTest<SuperStructureStates> s = SuperstructureBuilder.builder()
                                                            .addArms(arm, wrist).addMotors(rollers, algaeRollers)
                                                            .addSensors(new DigitalSensor("Intake", new DigitalInput(4), true))
                                                            .build();
  
  // public BooleanSupplier hasTarget;
  // public final Elevator elevator = new Elevator();
  // public final EndEffector endEffector = new EndEffector(arm, wrist, rollers, algaeRollers).setAutoEndScoring(false);
  // public Superstructure superstructure = new Superstructure(elevator, endEffector);
  // public CANdleSubsystem candle = new CANdleSubsystem(robotBase);
  // public static SuperstructureState selectedState = SuperstructureState.L4;
  public AutoAling alingLeft = new AutoAling(new GeneralAlign(robotBase, rController, controller, new Pose2d(Units.inchesToMeters(-20),Units.inchesToMeters(-6), new Rotation2d()), new Pose2d(Units.inchesToMeters(-10),Units.inchesToMeters(-6), new Rotation2d()), camLeft, camRight), robotBase, false);
  // public AutoAling alingRight = new AutoAling(new GeneralAlign(robotBase, rController, controller, new Pose2d(), new Pose2d(), camLeft, camRight), robotBase, true);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(0.15)
                                                              .setLeftSlewrate(1)
                                                              ;

                                                        
                                                              

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1).setSticksDeadzone(Constants.Controllers.STICK_DEADZONE); 
  public SendableChooser<Command> chooser;


  public double dist = 0;
  public double armSupplier = -92d;

  public double interpolate(Map<Double, Double> ogMap, double x) {
    
    HashMap<Double, Double> map = new HashMap<Double,Double>(ogMap);
    if (map == null || map.isEmpty()) return 0;

    
    List<Double> keys = new ArrayList<>(map.keySet());
    Collections.sort(keys);

    if (x <= keys.get(0)) return map.get(keys.get(0));
    if (x >= keys.get(keys.size() - 1)) return map.get(keys.get(keys.size() - 1));

    double lowerKey = keys.get(0);
    double upperKey = keys.get(keys.size() - 1);

    for (int i = 0; i < keys.size() - 1; i++) {
        double k1 = keys.get(i);
        double k2 = keys.get(i + 1);

        if (x >= k1 && x <= k2) {
            lowerKey = k1;
            upperKey = k2;
            break;
        }
    }

    double y1 = map.get(lowerKey);
    double y2 = map.get(upperKey);

    return y1 + (x - lowerKey) * ( (y2 - y1) / (upperKey - lowerKey) );
}

  public RobotContainer() 
  {
    // Enum<?> state = ArmState.Intaking;
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    robotBase.registerMechanism(arm, algaeRollers, wrist, rollers);
    robotBase.getLocalization().setSuppressUpdates(false);
    arm.setPidEnabled(true);
    wrist.setPidEnabled(true);
    arm.setFeedforwardEnabled(false);
    wrist.setFeedforwardEnabled(false);
    
    s.addActionableConstraint(new ActionableConstraint<SuperStructureStates>(SuperStructureStates.Intaking,SuperStructureStates.Score, () -> !s.getSensor("Intake").getSensorStatus()));
    s.addUpdateEvent(
    () -> 
    {
    double d = camLeft.getLimelight().getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    if(d > 0)
    {
      dist = d;
    }
    armSupplier = interpolate(Constants.EndEffector.distanceAndArm, dist);
    });
    // elevator.shuffleboard("Elevator");

    // //NEEED TOO DEPLOY 
    // //--------------------oFYUEIWHUIFWHEUIRHFIEU--------------///
    // hasTarget = () -> robotBase.getVision().getLimelight("limelight-left").hasValidTarget();
    
    // NamedCommands.registerCommand("WaitForTag", Commands.waitUntil(hasTarget));
    
    // NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.HomePID));
    NamedCommands.registerCommand("OrientLeftSide", new InstantCommand(() -> robotBase.getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(-2.3631872270622845)))));
    NamedCommands.registerCommand("OrientRightSide", new InstantCommand(() -> robotBase.getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(2.3631872270622845)))));
    NamedCommands.registerCommand("OrientMidSide", new InstantCommand(() -> robotBase.getLocalization().resetRelativePose(new Pose2d(0,0, Rotation2d.fromRadians(3.141592653589793)))));
    

    // NamedCommands.registerCommand("Intake", Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece()));

    // NamedCommands.registerCommand("L4", superstructure.setState(SuperstructureState.L4));
    // NamedCommands.registerCommand("L3", superstructure.setState(SuperstructureState.L3));
    // NamedCommands.registerCommand("L2", superstructure.setState(SuperstructureState.L2));
    // NamedCommands.registerCommand("L1", superstructure.setState(SuperstructureState.L1));
    // NamedCommands.registerCommand("AlgaeLow", superstructure.setState(SuperstructureState.AlgaeLow));


    // NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    // NamedCommands.registerCommand("WaitForElevator",superstructure.WaitForElevator());
    // NamedCommands.registerCommand("WaitForEffector",superstructure.WaitForL4());
    // NamedCommands.registerCommand("WaitForEjector", superstructure.WaitForEjector());

    // NamedCommands.registerCommand("AlignRight", alignRight);
    // NamedCommands.registerCommand("AlignLeft", alginLeft);
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


    chooser = Autos.AUTOS.createChooser(AUTOS.Left);
    SmartDashboard.putData(chooser);
  }

  // public boolean isIntaking()
  // {
  //   return superstructure.stateMachine.getGoalState().equals(SuperstructureState.Intaking);
  // }

  private void configureBindings() 
  {
    s.getSensor("Intake").getTrigger().onTrue(() -> s.setGoalState(SuperStructureStates.Home));
    driverController.b.onTrue(() -> s.setGoalState(SuperStructureStates.Aim));
    driverController.a.onTrue(() -> s.setGoalState(SuperStructureStates.Home));
  

    // driverController.rightStick.onTrue(new Aim(robotBase.getVision().getLimelight("limelight-left"), robotBase));
    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> {robotBase.getLocalization().resetFieldPose(0,0, 0); robotBase.getLocalization().resetRelativePose(0,0, 0);});

    // driverController.leftBumper.onTrue(
    //   Commands.either(
    //     Commands.either(superstructure.setState(SuperstructureState.Intaking), Commands.none(), () -> !endEffector.hasGamePiece())re
    //     superstructure.setState(SuperstructureState.HomePID),
    //     () -> !isIntaking()));

    // driverController.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    // driverController.pov.left.whileTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.Home));
    // driverController.pov.right.whileTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.Home));
   
    // driverController.rightTrigger.tiggerAt(0.5)
    // .onFalse(() -> 
    // {
    //   CommandScheduler.getInstance().cancel(alingRight); 
    //   // CommandScheduler.getInstance().schedule(superstructure.setState(SuperstructureState.Score));
    // }
    // );
    driverController.leftTrigger.tiggerAt(0.5).onTrue(()-> CommandScheduler.getInstance().schedule(alingLeft)).onFalse(() -> {CommandScheduler.getInstance().cancel(alingLeft);});
    // driverController.pov.down.onTrue(superstructure.setState(SuperstructureState.HomePID)).after(1).onTrue(superstructure.setState(SuperstructureState.Home));

    // driverController.a.onTrue(() -> selectedState = SuperstructureState.L1);//after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.b.onTrue(() -> selectedState = SuperstructureState.L2);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.x.onTrue(() -> selectedState = SuperstructureState.L3);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);
    // driverController.y.onTrue(() -> selectedState = SuperstructureState.L4);//.after(0.5).onTrue(() -> selectedState = SuperstructureState.Score);

    // //---------------------------------------------------------DRIVER 2--------------------------------------------------//

    // driverController2.a.onTrue(superstructure.setState(SuperstructureState.L1)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    // driverController2.b.onTrue(superstructure.setState(SuperstructureState.L2)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    // driverController2.x.onTrue(superstructure.setState(SuperstructureState.L3)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
    // driverController2.y.onTrue(superstructure.setState(SuperstructureState.L4)).after(0.75).onTrue(superstructure.setState(SuperstructureState.Score));
  
    // driverController2.start.whileTrue(superstructure.setState(SuperstructureState.AlgaeSpit)).after(1).onTrue(superstructure.setState(SuperstructureState.ScoreAlgae));
    // // driverController2.start.onTrue(() -> s.requestState2(S.Intaking.getSetpoint()));
    // driverController2.leftStick.and(driverController2.rightStick).onTrue(() -> {arm.setNudge(0); wrist.setNudge(0); elevator.resetNudge();});
    // driverController2.pov.up.onTrue(() -> elevator.nudge(1)).after(1).onTrue(() -> elevator.resetNudge());
    // driverController2.pov.down.onTrue(() -> elevator.nudge(-1)).after(1).onTrue(() -> elevator.resetNudge());
    driverController2.pov.right.onTrue(() -> arm.setNudge(arm.getNudge() + 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.pov.left.onTrue(() -> arm.setNudge(arm.getNudge() - 5)).after(1).onTrue(() -> arm.setNudge(0));
    driverController2.rightBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() + 5)).after(1).onTrue(() -> wrist.setNudge(0));
    driverController2.leftBumper.onTrue(() -> wrist.setNudge(wrist.getNudge() - 5)).after(1).onTrue(() -> wrist.setNudge(0));
    // driverController2.rightTrigger.tiggerAt(0.5).whileTrue(() -> {rollers.setMotors(1);}).onFalse(() -> rollers.setMotors(0));
    

  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
