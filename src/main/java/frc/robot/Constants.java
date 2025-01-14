package frc.robot;

import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMK4i;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {
    
    public interface DriveTrain {
        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
        double WHEELBASE_METERS = Units.inchesToMeters(18.375);

        String CANBUS = "can";
        SDSModules MODULE = new SDSModules(SDSMotor.FALCON_500, SDSMK4i.L1, CANBUS);
        PIDController ROTATION_PID = new PIDController(0.5 , 0, 0);
    
        PIDController DRIFT_PID = new PIDController(5.0, 0, 0);


        //SAM INFO
        //LF,FR,BL,BR
        int[] DRIVE_IDS = {1,2,3,4};
        int[] ROTATION_IDS = {5,6,7,8};
        int[] ENCODER_IDS = {9,10,11,12};
        double[] ENCODER_OFFSETS = {-0.5916,0,-0.0688132815 + 0.17749,0.36307 + 0.27};
        int PIGEON_ID = 20;

        //PATRICK INFO
        // LF,FR,BL,BR
        // int[] DRIVE_IDS = {5,14,6,17};
        // int[] ROTATION_IDS = {15,12,8,9};
        // int[] ENCODER_IDS = {1,2,3,4};
        // double[] ENCODER_OFFSETS = {0.3681 + 0.2639, 0.2026 - 0.4050 + 0.5, 0.1746 - 0.3491, -0.1140 + 0.2279 + 0.5};
        // int PIGEON_ID = 20;

        Translation2d[] MODULE_LOCATIONS = SwerveHelpers.generateModuleLocations(TRACKWIDTH_METERS, WHEELBASE_METERS);
        SwerveModuleConfig[] MODULE_CONFIGS = MODULE.generateConfigs(MODULE_LOCATIONS, DRIVE_IDS, ROTATION_IDS, ROTATION_PID, ENCODER_IDS, ENCODER_OFFSETS);


        String[] LIMELIGHTS = {"llm2"};
        RobotLocalizationConfig LOCALIZATION_CONFIG = new RobotLocalizationConfig(0.1, 0.1, 3);
    }

    public interface Controllers {
        double THETA_DEADZONE = 0.01;       
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
