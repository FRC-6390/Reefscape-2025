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
        SDSModules MODULE = new SDSModules(SDSMotor.KRAKEN_X60, SDSMK4i.L3, CANBUS);
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
        RobotLocalizationConfig LOCALIZATION_CONFIG = new RobotLocalizationConfig(0.1, 0.1, 0.001);
    }

    public interface Controllers {
        double THETA_DEADZONE = 0.1;      
    }
    public interface Climber {
    int LEFT_MOTOR = 0;
    int RIGHT_MOTOR = 1;
        
    }

    public interface Elevator {

        int ENCODER = 1;
        int LEFT_MOTOR = 2;
        int RIGHT_MOTOR = 3;
        double ROTATION_TO_CM = 320;
        int LOWER_LIMIT_SWITCH = 0;
        int UPPER_LIMIT_SWITCH = 0;
    
        enum Position {
            //ELEVATOR HEIGHT IN CM
            L1(0),
            L2(10),
            L3(20),
            L4(30),
            Feeder(15);

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
