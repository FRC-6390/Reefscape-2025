// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Align;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;

/** Add your docs here. */
public class AutomationCamera 
{
    public LimeLight ll;
    public PhotonCamera pc;

    public double xOffsetFromCenter;
    public double yOffsetFromCenter;
    public double heighOffGround;
    public double yaw;
    public double pitch;

    public AutomationCamera(LimeLight ll, double x, double y, double yaw, double pitch,double height)
    {
        this.ll = ll;
        this.xOffsetFromCenter = x;
        this.yOffsetFromCenter = y;
        this.yaw = yaw;
        this.pitch = pitch;
        this.heighOffGround = height;
    }

    public AutomationCamera(PhotonCamera pc, double x, double y, double yaw, double pitch, double height)
    {
        this.pc = pc;
        this.xOffsetFromCenter = x;
        this.yOffsetFromCenter = y;
        this.yaw = yaw;
        this.pitch = pitch;
        this.heighOffGround = height;
    }

    public double getYaw()
    {
        return yaw;
    }

    public double getPitch()
    {
        return pitch;
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

    public PhotonCamera getPhotonVision()
    {
        return pc;
    }

    public boolean isLimelight()
    {
        if(ll != null)
        {
            return true;
        }
        else{
        return false;
        }
    }

    public double getDistanceToTag()
    {
        if(isLimelight())
        {
            return ll.getPoseEstimate(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).getRaw()[9];
        }
        else
        {
            var result = pc.getAllUnreadResults().get(0);
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getBestCameraToTarget().getTranslation().getNorm();
         
        }
    }

    public double getTargetHorizontalOffset()
    {
        if(isLimelight())
        {
            return ll.getTargetHorizontalOffset();
        }
        else
        {
            var result = pc.getAllUnreadResults().get(0);
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getYaw();
         
        }
    }


    public double getTargetVerticalOffset()
    {
        if(isLimelight())
        {
            return ll.getTargetVerticalOffset();
        }
        else
        {
            var result = pc.getAllUnreadResults().get(0);
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getPitch();
         
        }
    }

    public boolean hasValidTarget()
    {
        if(isLimelight())
        {
            return ll.hasValidTarget();
        }
        else
        {
            return pc.getAllUnreadResults().get(0).hasTargets();
        }
    }

    public long getTagId()
    {
        if(isLimelight())
        {
            return ll.getAprilTagID();
        }
        else
        {
            var result = pc.getAllUnreadResults().get(0);
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getFiducialId();
        }
    }

    public double getHeight()
    {
        return heighOffGround;
    }

    
}
