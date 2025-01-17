// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToGoal extends Command {

  public SwerveDrivetrain drivetrain;
  public RobotLocalization localization;
  public double smallestDistance = 99999;
  public int desiredGoalIndex;
  public final String[] paths = 
  {
    "Side1",
    "Side2",
    "Side3",
    "Side4",
    "Side5",
    "Side6"
  };
  public Pose2d[] goals = 
  {
    //SIDE 1 PATH START POS
    new Pose2d(3.177,6.381,new Rotation2d()),
    //SIDE 2 PATH START POS
    new Pose2d(100000,100000,new Rotation2d()),
    //SIDE 3 PATH START POS
    new Pose2d(100000,100000,new Rotation2d()),
    //SIDE 4 PATH START POS
    new Pose2d(100000,100000,new Rotation2d()),
    //SIDE 5 PATH START POS
    new Pose2d(100000,100000,new Rotation2d()),
    //SIDE 6 PATH START POS
    new Pose2d(100000,100000,new Rotation2d())
  };
  /** Creates a new AprilTagAlign. */
  public DriveToGoal(SwerveDrivetrain drivetrain, RobotLocalization localization) {
    this.drivetrain = drivetrain;
    this.localization = localization;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    smallestDistance = 99999;
    for (int i = 0; i < goals.length; i++) {
      Transform2d distanceTransform = goals[i].minus(localization.getPose());
      double distance = Math.sqrt((distanceTransform.getY() * distanceTransform.getY() + distanceTransform.getX() * distanceTransform.getX()));
      if(distance <= smallestDistance)
      {
        smallestDistance = distance;
        desiredGoalIndex = i;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    try {
      CommandScheduler.getInstance().schedule(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(paths[desiredGoalIndex]), new PathConstraints(3, 3, 180, 540)));
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
