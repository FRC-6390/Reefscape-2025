// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import java.util.List;

import frc.robot.utils.Align.AlignCamera;


/** Add your docs here. */
public class PickupHelper 
{
    public List<AlignCamera> cams;

    public PickupHelper(AlignCamera... cams)
    {
        this.cams = List.of(cams);
    }

    public double calculateAngleToObject()
    {

        double r = 0;
        double count = 0;
        for (AlignCamera alignCamera : cams) {
            if(alignCamera.getLimelight().hasValidTarget())
                {
                    r += alignCamera.getLimelight().getTargetHorizontalOffset() + alignCamera.getYaw();
                    count++;
                }   
        }

        double rFinal = r / count;

        return r;
    }
}
