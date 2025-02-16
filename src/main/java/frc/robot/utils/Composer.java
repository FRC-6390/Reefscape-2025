// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

/** Add your docs here. */
public class Composer 
{
    public Composer()
    {
    }

    public static Command toDeadline(Command toCancel,Command command1, Command command2)
    {
        CommandScheduler.getInstance().cancel(toCancel);
        return new ParallelDeadlineGroup(command1, command2);
    }
}
