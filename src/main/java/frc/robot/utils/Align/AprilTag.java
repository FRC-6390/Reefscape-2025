// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public class AprilTag 
    {
        private final long redId;
        private final long blueId;

        private final Pose2d redPos;
        private final Pose2d bluePos;

        private final Rotation2d rotation;

    
        AprilTag(long redid,long blueid, Pose2d redPos, Pose2d bluePos, Rotation2d rotation) {
            this.rotation = rotation;
            this.redId = redid;
            this.blueId = blueid;
            this.redPos = redPos;
            this.bluePos = bluePos;
        }
        

        public Rotation2d getRotation2d()
        {
            return rotation;
        }

        public Pose2d getPose2d()
        {
            if(DriverStation.getAlliance().get().equals(Alliance.Red))
            {  
                return redPos;
            }
            else
            {
                return bluePos;
            }   
        }

        public long getId()
        {
            if(DriverStation.getAlliance().get().equals(Alliance.Red))
            {  
                return redId;
            }
            else
            {
                return blueId;
            }             
        }
        
    }