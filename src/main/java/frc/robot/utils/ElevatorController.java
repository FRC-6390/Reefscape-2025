// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ElevatorController 
{
    public PIDController feedbackcontroller;
    public ElevatorFeedforward elevatorFeedforward;
    public TrapezoidProfile motionProfile; 
    
    public ElevatorController(PIDController feedbackcontroller,ElevatorFeedforward elevatorFeedforward,TrapezoidProfile motionProfile)
    {
        this.feedbackcontroller = feedbackcontroller;
        this.elevatorFeedforward = elevatorFeedforward;
        this.motionProfile = motionProfile; 
    }

    public double calculateElevatorSpeed(double time, double setpoint, double currentMeasurment)
    {
        TrapezoidProfile.State profiledSetpoint = motionProfile.calculate(time, new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(setpoint, 0));
        return feedbackcontroller.calculate(currentMeasurment,profiledSetpoint.position) + elevatorFeedforward.calculate(profiledSetpoint.velocity) / 12;
    }
}
