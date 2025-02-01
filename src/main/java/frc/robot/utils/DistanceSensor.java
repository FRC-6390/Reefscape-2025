// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.nio.ByteBuffer;

import org.opencv.core.Range;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor;

/** Add your docs here. */
public class DistanceSensor extends Rev2mDistanceSensor {
    // private Rev2mDistanceSensor distanceSensor;

    
    public DistanceSensor(Port port)
    {
        super(port);
    }

    // public double getRange()
    // {
    //     return distanceSensor.getRange();
    // }
    // public double get()
    // {
    //     return distanceSensor.();
    // }
}

