// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  PowerDistribution pdh;
  public LimeLight lleft;
  public LimeLight llright;
  public Robot() {  
    m_robotContainer = new RobotContainer();
     lleft = m_robotContainer.robotBase.getVision().getLimelight("limelight-left");
     llright = m_robotContainer.robotBase.getVision().getLimelight("limelight-right");

    pdh = new PowerDistribution(14, ModuleType.kRev);
    m_robotContainer.robotBase.registerPIDCycles(this);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("LeftToTag", lleft.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
    SmartDashboard.putNumber("RightToTag", llright.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9]);
    SmartDashboard.putString("EmptyLabel", " ");

    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() 
  {
    pdh.clearStickyFaults();
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

  }

  @Override
  public void disabledInit() 
  {
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();

    m_robotContainer.superstructure.setSuper(SuperstructureState.Home);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.elevator.reset();
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
    m_robotContainer.elevator.reset();
    m_robotContainer.robotBase.resetPIDs();


    m_robotContainer.robotBase.getDrivetrain().getRobotSpeeds().stopSpeeds("auto");
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
