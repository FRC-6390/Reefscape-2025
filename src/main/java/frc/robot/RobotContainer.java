// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import au.grapplerobotics.LaserCan;
import frc.robot.commands.auto.ReefStrafe;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RelativeMove;
import frc.robot.commands.auto.AtState;
import frc.robot.commands.auto.AtStateEjector;
import frc.robot.commands.auto.PassiveAlign;
import frc.robot.commands.auto.RotateTo;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;
import frc.robot.commands.auto.DriveToPoint;

public class RobotContainer {

  public enum AUTOS {
        
        LEFTSIDE(new PathPlannerAuto("Choreo")),
        RIGHTSIDE(new PathPlannerAuto("ChoreoRight")),
        TESTLEFT(new PathPlannerAuto("ChoreoTestLeft")),
        TESTRIGHT(new PathPlannerAuto("ChoreoTestRight")),
        TESTMID(new PathPlannerAuto("ChoreoTestMid")),
        PRELOADLEFT(new PathPlannerAuto("PreLoadLeft")),
        PRELOADRIGHT(new PathPlannerAuto("PreLoadRight")),
        PRELOADMID(new PathPlannerAuto("PreLoadMid"));

        
        private final PathPlannerAuto auto;
    
        AUTOS(PathPlannerAuto auto){
            this.auto = auto;
        }

        public PathPlannerAuto getAuto()
        {
          return this.auto;
        }
    }

    public enum PATHS {
        
      SIDEA("SideA"),
      SIDEB("SideB"),

      SIDEC("SideC"),
      SIDED("SideD"),

      SIDEE("SideE"),
      SIDEF("SideF"),

      SIDEG("SideG"),
      SIDEH("SideH"),

      SIDEI("SideI"),
      SIDEJ("SideJ"),

      SIDEK("SideK"),
      SIDEL("SideL");


      
      private PathPlannerPath path;
      private final String pathName;
      
        
      PATHS(String pathName){
        this.pathName = pathName;
        try {
          this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            this.path =null;
          }
      }

      public PathPlannerPath getPath()
      {
        return path;
      }

      public String getPathName() {
          return pathName;
      }
  }

  public final LaserCan lasLeft = new LaserCan(1);
  public final LaserCan lasRight = new LaserCan(0);
  
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  public Elevator elevator = new Elevator();
  public EndEffector endEffector = new EndEffector(robotBase).setAutoEndScoring(true);
  public Superstructure superstructure = new Superstructure(elevator, endEffector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem(endEffector, superstructure);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(true)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(1.5);

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(false)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(1.5);

  public SendableChooser<String> chooser = new SendableChooser<>();
  // public Elevate elevate = new Elevate(ElevatorState.Home, lasLeft, lasRight, superstructure, robotBase, elevator);
  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    // robotBase.getLocalization().setAutoDrive((rs, chassis) -> {
    //   chassis.omegaRadiansPerSecond = -chassis.omegaRadiansPerSecond;
    //   rs.setAutoSpeeds(chassis);
    // });
    chooser.addOption("PreLoadLeft", "PreLoadLeft");
    chooser.addOption("PreLoadRight", "PreLoadRight");
    chooser.addOption("PreLoadMid", "PreLoadMid");
    chooser.addOption("ChoreoTestRight", "ChoreoTestRight");
    chooser.addOption("ChoreoTestMid", "ChoreoTestMid");
    chooser.addOption("ChoreoTestLeft", "ChoreoTestLeft");

    chooser.setDefaultOption("PreLoadLeft", "PreLoadLeft");


    SmartDashboard.putData(chooser);
    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    endEffector.shuffleboard("Effector");
    
    NamedCommands.registerCommand("Home", superstructure.setState(SuperstructureState.Home));
    NamedCommands.registerCommand("ManualL4", superstructure.setState(SuperstructureState.L4));
    NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    NamedCommands.registerCommand("WaitForElevator",Commands.race( new AtState(superstructure), new WaitCommand(3)));

    NamedCommands.registerCommand("WaitForEjector", Commands.race( new AtStateEjector(endEffector), new WaitCommand(3)));

    
    
    NamedCommands.registerCommand("RotateToRight",new RotateTo(robotBase,Rotation2d.fromRadians(-0.49333207719329186)));
    NamedCommands.registerCommand("RotateToLeft", new RotateTo(robotBase,Rotation2d.fromRadians(-2.575148734982150)));
    NamedCommands.registerCommand("RotateToMid", new RotateTo(robotBase,Rotation2d.fromRadians(-1.601756394242849)));

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
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0, Rotation2d.fromDegrees(0))));

    //PASSIVE ALIGN 
    // driverController.rightStick.toggleOnTrue(new PassiveAlign(robotBase));

    // //AUTO ALIGN (RIGHT BUMPER)
    
    // //EJECT PIECE MANUALLY
    driverController.rightBumper.whileTrue(superstructure.setState(SuperstructureState.Score));
    driverController.leftBumper.whileTrue(superstructure.setState(SuperstructureState.Score));

    
    // //SCORING COMMANDS
    driverController.a.onTrue(superstructure.setState(SuperstructureState.Home));
    driverController.b.onTrue(superstructure.setState(SuperstructureState.L2));
    driverController.x.onTrue(superstructure.setState(SuperstructureState.L3));
    driverController.y.onTrue(superstructure.setState(SuperstructureState.L4));

    // //ALGAE REMOVAL SEQUENCE
    driverController.rightTrigger.tiggerAt(0.5).onTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).onFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    driverController.leftTrigger.tiggerAt(0.5).onTrue(superstructure.setState(SuperstructureState.AlgaeLow)).onFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    
    //STRAFING
    driverController.pov.up.whileTrue(new ReefStrafe(lasLeft, lasRight, robotBase, -0.1,driverController));
    driverController.pov.down.whileTrue(new ReefStrafe(lasLeft, lasRight, robotBase, 0.1,driverController));

    // //----------------------------------------------------------DRIVER 2---------------------------------------------------------------//

    //FLIP EJECTION
    driverController2.leftBumper.onTrue(() -> {endEffector.getRollers().setFlip(true); endEffector.getRotator().setFlip(true);});
    driverController2.rightBumper.onTrue(() -> {endEffector.getRollers().setFlip(false); endEffector.getRotator().setFlip(false);});

    //END EFFECTOR OVERRIDE
    // driverController2.rightBumper.onTrue(superstructure.setState(SuperstructureState.Score));

    //ELEVATOR OVERIDE
    driverController2.pov.left.onTrue(superstructure.setElevator(ElevatorState.Home));
    driverController2.pov.up.onTrue(() -> elevator.nudge(1));
    driverController2.pov.down.onTrue(() -> elevator.nudge(-1));
  
    driverController2.start.after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0,Rotation2d.fromDegrees(90))));  
    // driverController2.leftBumper.whileTrue(() -> {robotBase.getRobotSpeeds().setAutoSpeeds(new ChassisSpeeds(0,0,0)); robotBase.getRobotSpeeds().stopAutoSpeeds();});
  }

  public Command getAutonomousCommand() 
  {
   return new PathPlannerAuto(chooser.getSelected()); 
  }
}
