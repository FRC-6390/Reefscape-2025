// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GoingBald;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Experimental.SuperStructureStates;
import frc.robot.subsystems.Experimental.SuperStructureTest;
import frc.robot.subsystems.Superstructure.SuperstructureTuple;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Aim extends Command {
  /** Creates a new Aim. */

  //DISTANCE IS THE X ARM ANGLE IS THE Y
  public double dist = 0;
  public HashMap<Double, Double> distanceAndArm = new HashMap<>();

  public AlignCamera cam;
  public SuperStructureTest s;

  public double interpolate(HashMap<Double, Double> map, double x) {
    
    if (map == null || map.isEmpty()) return 0;

    List<Double> keys = new ArrayList<>(map.keySet());
    Collections.sort(keys);

    if (x <= keys.get(0)) return map.get(keys.get(0));
    if (x >= keys.get(keys.size() - 1)) return map.get(keys.get(keys.size() - 1));

    double lowerKey = keys.get(0);
    double upperKey = keys.get(keys.size() - 1);

    for (int i = 0; i < keys.size() - 1; i++) {
        double k1 = keys.get(i);
        double k2 = keys.get(i + 1);

        if (x >= k1 && x <= k2) {
            lowerKey = k1;
            upperKey = k2;
            break;
        }
    }

    double y1 = map.get(lowerKey);
    double y2 = map.get(upperKey);

    return y1 + (x - lowerKey) * ( (y2 - y1) / (upperKey - lowerKey) );
}

  public Aim(AlignCamera cam, SuperStructureTest s) {
    this.s = s;
    this.cam = cam;
    distanceAndArm.put(1.85d, -72d);
    distanceAndArm.put(2.73d, -40d);
    distanceAndArm.put(3.64d, -15d);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    s.setGoalState(SuperStructureStates.Dynamic);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double d = cam.getLimelight().getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
    if(d > 0)
    {
      dist = d;
    }
    Robot.m_robotContainer.armSupplier = interpolate(distanceAndArm, dist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
