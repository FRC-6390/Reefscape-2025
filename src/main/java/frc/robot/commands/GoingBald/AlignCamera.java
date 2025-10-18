// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GoingBald;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;

/** Add your docs here. */
public class AlignCamera 
{
    public LimeLight ll;
    public double xOffsetFromCenter;
    public double yOffsetFromCenter;
    public double heighOffGround;
    public AlignCamera(LimeLight ll, double x, double y, double height)
    {
        this.ll = ll;
        this.xOffsetFromCenter = x;
        this.yOffsetFromCenter = y;
        this.heighOffGround = height;
    }

    public double getXOffset()
    {
        return xOffsetFromCenter;
    }

    public double getYOffset()
    {
        return yOffsetFromCenter;
    }

    public LimeLight getLimelight()
    {
        return ll;
    }

    public double getHeight()
    {
        return heighOffGround;
    }
}
