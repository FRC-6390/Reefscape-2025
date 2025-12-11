// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  // public V2 calibrate; //= new V2(m_robotContainer.robotBase, "limelight-right", false, m_robotContainer.superstructure, () -> m_robotContainer.selectedState);

  PowerDistribution pdh;
  public Robot() {  
    
    m_robotContainer = new RobotContainer();
    pdh = new PowerDistribution(14, ModuleType.kRev);
    m_robotContainer.robotBase.registerPIDCycles(this);
    // calibrate = new V2(m_robotContainer.robotBase, "limelight-right", false, m_robotContainer.superstructure, () -> m_robotContainer.selectedState);

  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.s.update();
    SmartDashboard.putNumber("Distance", m_robotContainer.robotBase.getVision().getLimelight("limelight-left").getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
  }

  @Override
  public void robotInit() 
  {
    pdh.clearStickyFaults();
    // m_robotContainer.alingLeft.align.shuffleboard();

    // m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();
    m_robotContainer.robotBase.getLocalization().resetRelativePose(new Pose2d());
    // SmartDashboard.putNumber("AutoAlign Y Offset", 15.5);
    // SmartDashboard.putNumber("AutoAlign X Offset Left", 11.5);
    // SmartDashboard.putNumber("AutoAlign X Offset Right", -6.2);
  }

  @Override
  public void disabledInit() 
  {
    // m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

    // m_robotContainer.superstructure.setSuper(SuperstructureState.Home);
  }

  @Override
  public void disabledPeriodic() 
  {
        // calibrate.GetController();
  }

  @Override
  public void disabledExit() 
  {
 }

  @Override
  public void autonomousInit() {
    // m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    Rotation2d offset = Rotation2d.fromDegrees(DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : 180);
    m_robotContainer.robotBase.getIMU().setVirtualAxis("driver", m_robotContainer.robotBase.getIMU().getVirtualAxis("field").minus(offset));
  }

  @Override
  public void teleopInit() {
    // m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();
m_robotContainer.robotBase.getLocalization().resetFieldPose(0, 0, 0);
    m_robotContainer.robotBase.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {
    
  }

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
