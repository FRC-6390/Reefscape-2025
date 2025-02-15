// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Test extends SubsystemBase {
//   /** Creates a new Test. */
//   SwerveDriveKinematics kinematics;
//   SwerveDriveOdometry odometry;
//   SwerveDrivetrain drivetrain;
//   public Test(SwerveDrivetrain drivetrain) 
//   {
//     this.drivetrain = drivetrain;
//     Translation2d m_frontLeftLocation = new Translation2d(0.2413, 0.2413);
//     Translation2d m_frontRightLocation = new Translation2d(0.2413, -0.2413);
//     Translation2d m_backLeftLocation = new Translation2d(-0.2413, 0.2413);
//     Translation2d m_backRightLocation = new Translation2d(-0.2413, -0.2413);
//     kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
//     odometry = new SwerveDriveOdometry(kinematics, drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions());

//   }


//   @Override
//   public void periodic() {
//     Pose2d pose = odometry.update(drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions());
//     SmartDashboard.putNumberArray("", null)
//     // This method will be called once per scheduler run
//   }
// }
