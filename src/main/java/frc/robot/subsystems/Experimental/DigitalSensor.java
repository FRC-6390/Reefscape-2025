// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Experimental;

import ca.frc6390.athena.commands.RunnableTrigger;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class DigitalSensor 
{
    public DigitalInput input;
    public boolean flipped;
    public String name;
    public DigitalSensor(String name,DigitalInput input, boolean flipped)
    {
        this.name = name;
        this.input = input;
        this.flipped = flipped;
    }

    public boolean isFlipped()
    {
        return flipped;
    }

    public RunnableTrigger getTrigger()
    {
        return new RunnableTrigger(() -> getSensorStatus());
    }

    public boolean getSensorStatus()
    {
        return isFlipped() ? !input.get() : input.get();
    }

    public String getName()
    {
        return name;
    }
}
