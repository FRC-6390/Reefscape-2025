// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public RobotConfig config;
  private AutoFactory factory;

 private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  public void drive(SwerveSample sample) {
     Pose2d pose = m_robotContainer.localization.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );



        // Apply the generated speeds
        m_robotContainer.driveTrain.drive(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, m_robotContainer.driveTrain.getIMU().getFieldYaw()));
  }


  public Robot() {  
    m_robotContainer = new RobotContainer();
headingController.enableContinuousInput(-Math.PI, Math.PI);
    factory = new AutoFactory(m_robotContainer.localization::getPose, m_robotContainer.localization::reset, this::drive, 
     true,
          m_robotContainer.driveTrain
          );

    // try{
    //   config = RobotConfig.fromGUISettings();  }catch(Exception e){
    //     DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    //   }
    //   System.out.println("AutoBuilder Configured");
    //   AutoBuilder.configure(
    //     m_robotContainer.localization::getPose, 
    //     m_robotContainer.localization::reset, 
    //     m_robotContainer.driveTrain::getDriveSpeeds, 
    //     (speeds, feedforwards) -> m_robotContainer.driveTrain.drive(speeds), 
    //     new PPHolonomicDriveController(
    //       new PIDConstants(5,0,0),
    //       new PIDConstants(5,0,0)
    //     ),
    //     config,
    //     () -> {
    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //     },
    //     m_robotContainer.driveTrain
    //   );
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // m_robotContainer.localization.update();
    // System.out.println(m_robotContainer.localization.getPose().getX());

    
    // SmartDashboard.putNumber("X", m_robotContainer.localization.getOdometryPose().getX());
    // SmartDashboard.putNumber("Y", m_robotContainer.localization.getOdometryPose().getY());
    // SmartDashboard.putNumber("Theta", m_robotContainer.localization.getOdometryPose().getRotation().getDegrees());
  }

  @Override
  public void robotInit() {
    // m_robotContainer.driveTrain.shuffleboard("DriveTrain");
    // m_robotContainer.localization.shuffleboard("Localization");
    // m_robotContainer.localization.reset();
    // m_robotContainer.localization.reset(new Pose2d(0,0, new Rotation2d()));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    factory.resetOdometry("test");
    m_autonomousCommand = factory.trajectoryCmd("test");

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
