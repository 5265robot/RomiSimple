// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    // Trajectory paths -- ALL DISTANCES IN meters (NOT inches, not centimeters)
    public final static class path{
        public static class A {            
            static final Pose2d startPose = new Pose2d(0.0,0.0,new Rotation2d(Units.degreesToRadians(0)));
            static final Pose2d endPose = new Pose2d(0.8,0.8,new Rotation2d(Units.degreesToRadians(90)));
            static List<Translation2d> points = List.of(
                //new Translation2d(0.02, 0.01),
                //new Translation2d(0.06, 0.02),
                //new Translation2d(0.08, 0.03)
            
                //new Translation2d(1.1, 0.0),
                //new Translation2d(2.2, 0.0),
                new Translation2d(0.4, 0.4)
                //new Translation2d(0.16, 0.1),
                //new Translation2d(0.17, 0.14)
                //new Translation2d(0.17, 0.0)
                );
        }
        public static class B{            
            static final Pose2d startPose = new Pose2d(0.0,0.0,new Rotation2d(Units.degreesToRadians(90)));
            static final Pose2d endPose = new Pose2d(0.0,0.2,new Rotation2d(Units.degreesToRadians(90)));
            static List<Translation2d> points = List.of(
                new Translation2d(0.2,0.0),
                new Translation2d(0.2,0.2)
                );
        }
        public static class C{            
            static final Pose2d startPose = new Pose2d(0.0,0.0,new Rotation2d(Units.degreesToRadians(90)));
            static final Pose2d endPose = new Pose2d(0.0,0.2,new Rotation2d(Units.degreesToRadians(90)));
            static List<Translation2d> points = List.of(
                new Translation2d(0.2,0.0),
                new Translation2d(0.2,0.2)
                );
        }
        public static class D{            
            static final Pose2d startPose = new Pose2d(0.0,0.0,new Rotation2d(Units.degreesToRadians(90)));
            static final Pose2d endPose = new Pose2d(0.0,0.2,new Rotation2d(Units.degreesToRadians(90)));
            static List<Translation2d> points = List.of(
                new Translation2d(0.2,0.0),
                new Translation2d(0.2,0.2)
                );
        }
    }

    // Trajectory
    public final static class traj {

        // Feedforward Analysis: kS
        public static final double ksVolts = 0.353;
        // Feedforward Analysis: kV
        public static final double kvVoltSecondsPerMeter = 2.21;
        // Feedforward Analysis: kA
        public static final double kaVoltSecondsSquaredPerMeter = 0.0381;

        // check this width
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
        // get this from characterization routine!!
        // Optimal Controller Gain for preset "WPILib (2020-)": kP
        public static final double kPDriveVel = 0.691;
    }

    // RomiInput variables
    public final class AnalogInputs {
        // input position on the Romi for the light sensor
        public static final int lightSensor = 0;
    }

    // Input Output variables
    public final class IO {
        // xBox controller port
        public static final int Controller = 0;
        public final XboxController.Button buttonB = XboxController.Button.kB;

    }

    // Driving variables
    public final class Drive {
        // how fast?
        public static final double speed = 1.0;
        // curve drive quick turn?
        public static final boolean qTurn = false;

        // subset for auto
        public final class Auto {
            public static final double simpleDriveSpeed = 0.5;
            public static final double kSpeed = 0.8;
            public static final double kAccel = 0.8;

            public static final double simpleDriveTime = 2.0;
            public static final double simpleDriveTurn = 1.0;
            public static final double simpleSpinVolt = 0.5;
			public static final double simpleSpinTimeout = 1.0;
        }
        // line followers
        // level to keep while driving
        public static final double lightLevel = 0.5;
        // which side of the line? +1 is right, -1 is left
        public static final double lineSide = +1.0;
        // proportional, integral and derivative
        public static final double kLineP = 2.4, kLineI = 2.0, kLineD = 0.0;
        // was 4.2, 1, 5
        // threshold for turning
        public static final double diffSquare = 0.007;
        // turnrate inflection
        public static final double inflection = 2.7;

    }
}
