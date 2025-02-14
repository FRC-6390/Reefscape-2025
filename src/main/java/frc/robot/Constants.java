package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveHelpers;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMK4i;
import ca.frc6390.athena.drivetrains.swerve.modules.SDSModules.SDSMotor;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

        PIDConstants PATHPLANNER_TRANSLATION_PID = new PIDConstants(5,0,0);
        PIDConstants PATHPLANNER_ROTATION_PID = new PIDConstants(5,0,0);
    }

    public interface Controllers {
        double THETA_DEADZONE = 0.15;      
    }
    public interface Climber {
        int LIMIT_SWITCH = 1;
        int ENCODER = 40;
        int LEFT_MOTOR = 21;
        int RIGHT_MOTOR = 30;
        double ENCODER_OFFSET = 0;
        double ENCODER_GEAR_RATIO = 4d/1d;
        String CANBUS = "can";
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);
    }

    public interface Elevator {
        String CANBUS = "rio";
        int ENCODER = 23;
        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 2d;
        double OFFSET_FROM_FLOOR = 0;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 6d/1d;
        int LIMIT_SWITCH = 0;
        //MAX ACCEL WAS 15
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.01, 0, new Constraints(60, 18));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.17, 0.377,0.78);
        
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0, 0, new Constraints(60, 30));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.129, 0.377,0.75);
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.2, 0, 0, new Constraints(0.5, 0.5));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.12875, 0.688,0.00);

        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.0095, 0, new Constraints(0.5, 0), 5);
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.04, 2.67,0.006666667);
    }

    public interface EndEffector {
        int LIMIT_SWITCH = 1;
        int ENCODER = 40;
        int LEFT_MOTOR = 21;
        int RIGHT_MOTOR = 30;
        double ENCODER_OFFSET = 0;
        double ENCODER_GEAR_RATIO = 1d/1d; //from motors 125d/1d;
        String CANBUS = "can";
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);

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
