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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.Elevate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.CANdleSubsystem;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.EndEffector.AlgaeExtensionState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;

public class RobotContainer {

  public enum AUTOS {
        
        LEFTSIDE(new PathPlannerAuto("Choreo")),
        RIGHTSIDE(new PathPlannerAuto("ChoreoRight")),
        TEST(new PathPlannerAuto("ChoreoTest"));
        
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
  public Elevator elevator = new Elevator();
  public EndEffector effector = new EndEffector();
  
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  public Superstructure superstructure = new Superstructure(elevator, effector, robotBase);
  public CANdleSubsystem candle = new CANdleSubsystem(effector, robotBase, superstructure);
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
    // robotBase.getDrivetrain().setDriveCommand(driverController);
    robotBase.getLocalization().setAutoDrive((rs, chassis) -> {
      chassis.omegaRadiansPerSecond = -chassis.omegaRadiansPerSecond;
      rs.setAutoSpeeds(chassis);
    });
    chooser.addOption("LEFT SIDE", AUTOS.LEFTSIDE);
    chooser.addOption("RIGHT SIDE", AUTOS.RIGHTSIDE);
    chooser.addOption("TEST", AUTOS.TEST);
    chooser.setDefaultOption("TEST", AUTOS.TEST);

    SmartDashboard.putData(chooser);
    elevator.shuffleboard("Elevator");
    // elevator.setDefaultCommand(elevate);
    effector.shuffleboard("Effector");
    

    // NamedCommands.registerCommand("L4", new InstantCommand(() -> elevate.setState(ElevatorState.L4)));
    // NamedCommands.registerCommand("L1", new InstantCommand(() -> elevate.setState(ElevatorState.L1)));
    NamedCommands.registerCommand("Home", superstructure.setElevator(ElevatorState.Home));
    
  }

  private void configureBindings() 
  {

    // driverController.leftBumper.whileTrue(() -> elevator.setMotors(1)).onFalse(() -> elevator.setMotors(0));
    // driverController.rightBumper.whileTrue(() -> elevator.setMotors(-0.1)).onFalse(() -> elevator.setMotors(0));
    
    // driverController.pov.up.onTrue(() -> elevator.setCurrentLimit(elevator.getCurrentLimit()+0.5));
    // driverController.pov.down.onTrue(() -> elevator.setCurrentLimit(elevator.getCurrentLimit()-0.5));

    // driverController.a.onTrue(superstructure.setAlgaeMachine(AlgaeExtensionState.Extended));
    // // driverController.b.onTrue(superstructure.setAlgaeMachine(AlgaeExtensionState.Home));
    // driverController.rightBumper.onTrue(superstructure.setElevator(ElevatorState.Home));

    // driverController.a.onTrue(superstructure.setElevator(ElevatorState.L1));
    // driverController.b.onTrue(superstructure.setElevator(ElevatorState.L2));
    // driverController.y.onTrue(superstructure.setElevator(ElevatorState.L3));
    // driverController.x.onTrue(superstructure.setElevator(ElevatorState.L4));

    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    //RESET ODOMETRY
    // driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0));

    // //PASSIVE ALIGN (RIGHT STICK)
    // driverController.rightStick.toggleOnTrue(new PassiveAlign(robotBase, las));

    // //AUTO ALIGN (RIGHT BUMPER)
    // driverController.rightBumper.whileTrue(new DriveToPoint(robotBase));

    // //EJECT PIECE MANUALLY
    driverController.leftBumper.whileTrue(() -> superstructure.ejectPiece(1)).whileTrue(() -> superstructure.ejectPiece(0));

    // //SCORING COMMANDS
    driverController.a.onTrue(superstructure.setElevator(ElevatorState.Home));
    driverController.b.onTrue(superstructure.setElevator(ElevatorState.L2));
    driverController.x.onTrue(superstructure.setElevator(ElevatorState.L3));
    driverController.y.onTrue(superstructure.setElevator(ElevatorState.L4));
    // //ALGAE REMOVAL SEQUENCE
    driverController.pov.up.onTrue(superstructure.setAlgaeMachine(AlgaeExtensionState.Extended));
    driverController.pov.down.onTrue(superstructure.setAlgaeMachine(AlgaeExtensionState.Home));
 

    // //----------------------------------------------------------DRIVER 2---------------------------------------------------------------//


    // //EFFECTOR OVERRIDE
    driverController2.a.onTrue(superstructure.setEndEffector(EndEffectorState.Left));
    driverController2.b.onTrue(superstructure.setEndEffector(EndEffectorState.LeftL4));
    driverController2.y.onTrue(superstructure.setEndEffector(EndEffectorState.Right));
    driverController2.x.onTrue(superstructure.setEndEffector(EndEffectorState.RightL4));
    driverController2.rightBumper.onTrue(superstructure.setEndEffector(EndEffectorState.Home));


    // // ELEVATOR NUDGING
    driverController2.pov.up.onTrue(() -> elevator.nudge(1));
    driverController2.pov.down.onTrue(() -> elevator.nudge(-1));
    
    // // RESET FIELD ODOMETRY
    driverController2.start.after(2).onTrue(() -> robotBase.getLocalization().resetFieldPose(new Pose2d(0,0,new Rotation2d())));
  
    
  }

  public Command getAutonomousCommand() 
  {
   return chooser.getSelected().getAuto(); 
  }
}
