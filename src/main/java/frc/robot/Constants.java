package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMK4i;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMotor;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    
    public interface DriveTrain {
        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
        double WHEELBASE_METERS = Units.inchesToMeters(18.375);

        String CANBUS = "can";
        SDSModules MODULE = new SDSModules(SDSMotor.FALCON_500, SDSMK4i.L3, CANBUS);
        PIDController ROTATION_PID = new PIDController(0.5 , 0,0);
    
        PIDController DRIFT_PID = new PIDController(1.5, 0, 0);

        int PIGEON_ID = 20;
        //LF,FR,BL,BR
        int[] DRIVE_IDS = {1,2,3,4};
        int[] ROTATION_IDS = {5,6,7,8};
        int[] ENCODER_IDS = {9,10,11,12};
        // SAM OFFSETS
        // double[] ENCODER_OFFSETS = {0.697,3.123,2.445,-0.851};
        // PATRICK OFFSETS
        double[] ENCODER_OFFSETS = {2.284,-1.942,1.0952,2.376};
        

        Translation2d[] MODULE_LOCATIONS = SwerveHelpers.generateModuleLocations(TRACKWIDTH_METERS, WHEELBASE_METERS);
        SwerveModuleConfig[] MODULE_CONFIGS = MODULE.generateConfigs(MODULE_LOCATIONS, DRIVE_IDS, ROTATION_IDS, ROTATION_PID, ENCODER_IDS, ENCODER_OFFSETS);

        String[] LIMELIGHTS = {"limelight-driver", "limelight-tag"};
        RobotLocalizationConfig LOCALIZATION_CONFIG = new RobotLocalizationConfig(3333, 33333, 33333, 0.000001, 0.0000001,0.0000001, PoseEstimateWithLatencyType.BOT_POSE_BLUE);
    }

    public interface Controllers {
        double THETA_DEADZONE = 0.1;      
    }

    public interface Elevator {
    
        enum Position {
            L1(-1),
            L2(-1),
            L3(-1),
            L4(-1),
            Feeder(-1);

            double pos;
            private Position(double pos){
                this.pos = pos;
            }

            public double getPosition() {
                return pos;
            }
        }
    }

   

    public interface FieldElements {

        enum ReefAprilTag {
            Red(-1),
            Blue(-1);
            double pos;
            private ReefAprilTag(double pos){
                this.pos = pos;
            }


            public double getPosition() {
                return pos;
            }
        }

    }
}
