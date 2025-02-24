// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.security.PublicKey;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.commands.AutoCommands;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.DriveToPoint;
import frc.robot.commands.auto.PassiveAlign;
import frc.robot.commands.mechanisms.Elevate;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.EndEffector;
import frc.robot.subsystems.superstructure.Climber.ClimberState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector.AlgaeExtensionState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class RobotContainer {

  public enum AUTOS {
        
        LEFTSIDE(new PathPlannerAuto("Choreo"));
        
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
      SIDEC("SideC"),
      SIDEE("SideE"),
      SIDEG("SideG"),
      SIDEI("SideI"),
      SIDEK("SideK");

      
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
  }

  public final LaserCan las = new LaserCan(59);
  public Elevator elevator = new Elevator();
  public Climber climber = new Climber();
  public EndEffector effector = new EndEffector();
  public Superstructure superstructure = new Superstructure(climber, elevator, effector);
  public final RobotBase<SwerveDrivetrain> robotBase = Constants.DriveTrain.ROBOT_BASE.create().shuffleboard();
  private final EnhancedXboxController driverController = new EnhancedXboxController(0)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(3.5);

  private final EnhancedXboxController driverController2 = new EnhancedXboxController(1)
                                                              .setLeftInverted(false)
                                                              .setRightInverted(true)
                                                              .setSticksDeadzone(Constants.Controllers.STICK_DEADZONE)
                                                              .setLeftSlewrate(3.5);
  public SendableChooser<AUTOS> chooser = new SendableChooser<>();
  public Elevate l1 = new Elevate(ElevatorState.L1, las, superstructure, robotBase);
  public Elevate l2 = new Elevate(ElevatorState.L2, las, superstructure, robotBase);
  public Elevate l3 = new Elevate(ElevatorState.L3, las, superstructure, robotBase);
  public Elevate l4 = new Elevate(ElevatorState.L4, las, superstructure, robotBase);

  public RobotContainer() 
  {
    configureBindings();
    robotBase.getDrivetrain().setDriveCommand(driverController);
    chooser.addOption("LEFT SIDE", AUTOS.LEFTSIDE);
    SmartDashboard.putData(chooser);
    elevator.shuffleboard("Elevator");
    climber.shuffleboard("Climber");
    effector.shuffleboard("Effector");

    NamedCommands.registerCommand("L4", new Elevate(ElevatorState.L4, las, superstructure, robotBase));
    NamedCommands.registerCommand("L3", new Elevate(ElevatorState.L3, las, superstructure, robotBase) );
    NamedCommands.registerCommand("L2", new Elevate(ElevatorState.L2, las, superstructure, robotBase));
    NamedCommands.registerCommand("L1", new Elevate(ElevatorState.L1, las, superstructure, robotBase));
    NamedCommands.registerCommand("Feeder", superstructure.setElevator(ElevatorState.Feeder));
    NamedCommands.registerCommand("Home", superstructure.setElevator(ElevatorState.Home));
    NamedCommands.registerCommand("StartConfiguration", superstructure.setElevator(ElevatorState.StartConfiguration)); 
    
  }

  private void configureBindings() 
  {

    //----------------------------------------------------------DRIVER 1---------------------------------------------------------------//

    //RESET ODOMETRY
    driverController.start.onTrue(() -> robotBase.getDrivetrain().getIMU().setYaw(0)).after(3).onTrue(() -> robotBase.getLocalization().resetFieldPose(0,0,0));
    
    //PASSIVE ALIGN (RIGHT STICK)
    driverController.rightStick.toggleOnTrue(new PassiveAlign(robotBase, las));

    //AUTO ALIGN (RIGHT BUMPER)
    driverController.rightBumper.whileTrue(new DriveToPoint(robotBase , las));

    //EJECT PIECE MANUALLY
    driverController.leftBumper.onTrue(() -> superstructure.ejectPiece(1)).onFalse(() -> superstructure.ejectPiece(0));

    //SCORING COMMANDS
    driverController.a.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l2, l3, l4)), l1));
    driverController.b.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l3, l4)), l2));
    driverController.y.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l2, l1, l4)), l3));
    driverController.x.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l2, l3, l1)), l4));

    //ALGAE REMOVAL SEQUENCE
    driverController.pov.up.toggleOnTrue(Commands.sequence(superstructure.setElevator(ElevatorState.Feeder), superstructure.setAlgaeMachine(AlgaeExtensionState.Extended),  new InstantCommand(() -> superstructure.ejectPiece(1))));
    driverController.pov.up.toggleOnFalse(superstructure.setElevator(ElevatorState.L4));
    driverController.pov.up.toggleOnFalse(Commands.sequence(superstructure.setAlgaeMachine(AlgaeExtensionState.Home),  new InstantCommand(() -> superstructure.ejectPiece(0)))).after(0.35);
 
    //CLIMB
    driverController.pov.down.onTrue(superstructure.setClimber(ClimberState.Climb));

    //----------------------------------------------------------DRIVER 2---------------------------------------------------------------//

    //ELEVATOR OVERRIDE
    driverController2.a.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)), superstructure.setElevator(ElevatorState.L1)));
    driverController2.b.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setElevator(ElevatorState.L2)));
    driverController2.y.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setElevator(ElevatorState.L3)));
    driverController2.x.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setElevator(ElevatorState.L4)));

    //EFFECTOR OVERRIDE
    driverController2.a.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)), superstructure.setEndEffectir(EndEffectorState.Left)));
    driverController2.b.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setEndEffectir(EndEffectorState.LeftL4)));
    driverController2.y.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setEndEffectir(EndEffectorState.Right)));
    driverController2.x.onTrue(Commands.sequence(new InstantCommand(() -> CommandScheduler.getInstance().cancel(l1, l2, l3, l4)),superstructure.setEndEffectir(EndEffectorState.RightL4)));

    
  }
  public Command getAutonomousCommand() 
  {
   return chooser.getSelected().getAuto(); 
  }
}
