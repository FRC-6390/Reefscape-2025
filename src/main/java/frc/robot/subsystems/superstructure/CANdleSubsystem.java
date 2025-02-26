// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;

public class CANdleSubsystem extends SubsystemBase {
  /** Creates a new CANdle. */
  public EndEffector effector;
  public RobotBase<?> base;
  public Superstructure superstructure;
  public CANdleSubsystem(EndEffector effector, RobotBase<?> base, Superstructure superstructure) 
  {
    this.effector = effector;
    this.base = base;
    this.superstructure = superstructure;
  }

  @Override
  public void periodic() {
    effector.candle.setLEDs((effector.beamBreakCenter.getAsBoolean() ? 0 : 255), (effector.beamBreakCenter.getAsBoolean() ? 255 : 0), (superstructure.closeEnough() ? 255 : 0));
    // This method will be called once per scheduler run
  }
}
