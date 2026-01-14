// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Align;

import org.photonvision.PhotonCamera;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVision;

/** Add your docs here. */
public class AlignCamera 
{
    public LimeLight ll;
    public PhotonVision pc;

    public double xOffsetFromCenter;
    public double yOffsetFromCenter;
    public double heighOffGround;
    public double yaw;
    public AlignCamera(LimeLight ll, double x, double y, double yaw, double height)
    {
        this.ll = ll;
        this.xOffsetFromCenter = x;
        this.yOffsetFromCenter = y;
        this.yaw = yaw;
        this.heighOffGround = height;
    }

    public AlignCamera(PhotonVision pc, double x, double y, double yaw, double height)
    {
        this.pc = pc;
        this.xOffsetFromCenter = x;
        this.yOffsetFromCenter = y;
        this.yaw = yaw;
        this.heighOffGround = height;
    }

    public double getYaw()
    {
        return yaw;
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

    public PhotonVision getPhotonVision()
    {
        return pc;
    }

    public double getHeight()
    {
        return heighOffGround;
    }
}
