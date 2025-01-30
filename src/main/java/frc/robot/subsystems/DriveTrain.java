// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import ca.frc6390.athena.core.RobotIMU;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.Constants;

public class DriveTrain extends SwerveDrivetrain {


  /** Creates a new DriveTrain. */
  public DriveTrain(RobotIMU imu) {
    super(Constants.DriveTrain.MODULE_CONFIGS,imu, false, Constants.DriveTrain.DRIFT_PID);    
  }

  @Override
  public void periodic() {
    this.update();
  }
}
