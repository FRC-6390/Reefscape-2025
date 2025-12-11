// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Interpolation;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class Logger 
{
    public DoubleSupplier supplier;
    public String name;
    public Logger(String name,DoubleSupplier supplier)
    {
        this.name = name;
        this.supplier = supplier;
    }

    public double getValue()
    {
        return supplier.getAsDouble();
    }

    public String getName()
    {
        return name;
    }
}
