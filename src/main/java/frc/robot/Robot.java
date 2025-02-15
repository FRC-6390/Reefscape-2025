// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {  
    m_robotContainer = new RobotContainer();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    }

  @Override
  public void robotPeriodic() {
    m_robotContainer.robotBase.getLocalization().update();
    System.out.println(m_robotContainer.robotBase.getLocalization().getFieldPose());
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {}

  @Override
  public void disabledInit() 
  {
    // m_robotContainer.elevator.reset(elevator.getHeight());
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.elevator.reset(elevator.getHeight());
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
    m_robotContainer.robotBase.getDrivetrain().getRobotSpeeds().setAutoSpeeds(new ChassisSpeeds());
    // m_robotContainer.elevator.reset(elevator.getHeight());
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
    // m_robotContainer.elevator.reset(elevator.getHeight());
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
