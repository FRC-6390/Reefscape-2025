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
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.PassiveAlign;
import frc.robot.commands.auto.RotateTo;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;

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
  public EndEffector endEffector = new EndEffector(robotBase);
  public Superstructure superstructure = new Superstructure(elevator, endEffector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem(endEffector, superstructure);

  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(false)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(1.5);

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(false)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(1.5);

  public SendableChooser<AUTOS> chooser = new SendableChooser<>();
  // public Elevate elevate = new Elevate(ElevatorState.Home, lasLeft, lasRight, superstructure, robotBase, elevator);
  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    // robotBase.getLocalization().setAutoDrive((rs, chassis) -> {
    //   chassis.omegaRadiansPerSecond = -chassis.omegaRadiansPerSecond;
    //   rs.setAutoSpeeds(chassis);
    // });
    chooser.addOption("LEFT SIDE", AUTOS.LEFTSIDE);
    chooser.addOption("RIGHT SIDE", AUTOS.RIGHTSIDE);
    chooser.addOption("TESTLEFT", AUTOS.TESTLEFT);
    chooser.addOption("TESTRIGHT", AUTOS.TESTRIGHT);
    chooser.addOption("TESTMID", AUTOS.TESTMID);
    chooser.addOption("PRELOADLEFT", AUTOS.PRELOADLEFT);
    chooser.addOption("PRELOADRIGHT", AUTOS.PRELOADRIGHT);
    chooser.setDefaultOption("PRELOADMID", AUTOS.PRELOADMID);




    SmartDashboard.putData(chooser);
    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    endEffector.shuffleboard("Effector");
    
    NamedCommands.registerCommand("Home", Commands.sequence(Commands.sequence(superstructure.setElevator(ElevatorState.Home), superstructure.setEndEffector(EndEffectorState.Home))));
    NamedCommands.registerCommand("ManualL4", Commands.sequence(superstructure.setState(SuperstructureState.L4), superstructure.setState(SuperstructureState.Score)));
    NamedCommands.registerCommand("StartEject", superstructure.setState(SuperstructureState.Score));
    NamedCommands.registerCommand("Align", new PassiveAlign(robotBase));

    
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
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0, Rotation2d.fromDegrees(180))));

    //PASSIVE ALIGN 
    // driverController.rightStick.toggleOnTrue(new PassiveAlign(robotBase));

    // //AUTO ALIGN (RIGHT BUMPER)
    // driverController.rightBumper.onTrue(new DriveToPoint(robotBase));


    // //EJECT PIECE MANUALLY
    driverController.leftBumper.whileTrue(() -> superstructure.setState(SuperstructureState.Score));
    
    // //SCORING COMMANDS
    driverController.a.onTrue(superstructure.setElevator(ElevatorState.Home));
    driverController.b.onTrue(superstructure.setElevator(ElevatorState.L2)).toggleOnFalse(superstructure.setElevator(ElevatorState.Home));
    driverController.x.onTrue(superstructure.setElevator(ElevatorState.L3)).toggleOnFalse(superstructure.setElevator(ElevatorState.Home));
    driverController.y.onTrue(superstructure.setElevator(ElevatorState.L4)).toggleOnFalse(superstructure.setElevator(ElevatorState.Home));

    // //ALGAE REMOVAL SEQUENCE
    driverController.pov.up.toggleOnTrue(superstructure.setState(SuperstructureState.AlgaeHigh)).toggleOnFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    driverController.pov.down.toggleOnTrue(superstructure.setState(SuperstructureState.AlgaeLow)).toggleOnFalse(superstructure.setState(SuperstructureState.AlgaeRetract));
    
    //STRAFING
    driverController.pov.left.whileTrue(() -> robotBase.getRobotSpeeds().setFeedbackSpeeds(0,-0.3,0)).onFalse(() -> robotBase.getRobotSpeeds().stopFeedbackSpeeds());
    driverController.pov.right.whileTrue(() -> robotBase.getRobotSpeeds().setFeedbackSpeeds(0,0.3,0)).onFalse(() -> robotBase.getRobotSpeeds().stopFeedbackSpeeds());
    driverController.pov.down.whileTrue(() -> robotBase.getRobotSpeeds().setFeedbackSpeeds(-0.3,0,0)).onFalse(() -> robotBase.getRobotSpeeds().stopFeedbackSpeeds());
    driverController.pov.up.whileTrue(() -> robotBase.getRobotSpeeds().setFeedbackSpeeds(0.3,0,0)).onFalse(() -> robotBase.getRobotSpeeds().stopFeedbackSpeeds());

    // //----------------------------------------------------------DRIVER 2---------------------------------------------------------------//

    //FLIP EJECTION
    driverController2.a.onTrue(() -> endEffector.getRollers().setFlip(true));
    driverController2.b.onTrue(() -> endEffector.getRollers().setFlip(false));

    driverController2.x.onTrue(() -> endEffector.getRotator().setFlip(true));
    driverController2.y.onTrue(() -> endEffector.getRotator().setFlip(false));

    //END EFFECTOR OVERRIDE
    driverController2.x.onTrue(superstructure.setEndEffector(EndEffectorState.L4));
    driverController2.y.onTrue(superstructure.setEndEffector(EndEffectorState.L4));
    driverController2.rightBumper.onTrue(superstructure.setEndEffector(EndEffectorState.Home));

    //ELEVATOR OVERIDE
    driverController2.pov.left.onTrue(superstructure.setElevator(ElevatorState.Home));
    driverController2.pov.up.onTrue(() -> elevator.nudge(1));
    driverController2.pov.down.onTrue(() -> elevator.nudge(-1));
  
    driverController2.start.after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0,new Rotation2d())));  
  }

  public Command getAutonomousCommand() 
  {
   return chooser.getSelected().getAuto(); 
  }
}
