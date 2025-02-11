// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ElevatorController 
{
    public ProfiledPIDController feedbackcontroller;
    public ElevatorFeedforward elevatorFeedforward;
    
    public ElevatorController(ProfiledPIDController feedbackcontroller,ElevatorFeedforward elevatorFeedforward )
    {
        this.feedbackcontroller = feedbackcontroller;
        this.elevatorFeedforward = elevatorFeedforward;
    }

    public double calculateElevatorSpeed(double setpoint, double currentMeasurment)
    {
        return feedbackcontroller.calculate(currentMeasurment,setpoint) + elevatorFeedforward.calculate(feedbackcontroller.getSetpoint().velocity) / 12;
    }
}
