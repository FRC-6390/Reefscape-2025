// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.V2;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  public V2 calibrate; //= new V2(m_robotContainer.robotBase, "limelight-right", false, m_robotContainer.superstructure, () -> m_robotContainer.selectedState);

  PowerDistribution pdh;
  public Robot() {  
    
    m_robotContainer = new RobotContainer();
    pdh = new PowerDistribution(14, ModuleType.kRev);
    m_robotContainer.robotBase.registerPIDCycles(this);
    calibrate = new V2(m_robotContainer.robotBase, "limelight-right", false, m_robotContainer.superstructure, () -> m_robotContainer.selectedState);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() 
  {
    pdh.clearStickyFaults();
    m_robotContainer.robotBase.getLocalization().resetRelativePose(0, 0, 0);
  }

  @Override
  public void disabledInit() 
  {
    m_robotContainer.superstructure.setSuper(SuperstructureState.Home);
  }

  @Override
  public void disabledPeriodic() 
  {
        calibrate.GetController();
  }

  @Override
  public void disabledExit() 
  {
 }

  @Override
  public void autonomousInit() {
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
