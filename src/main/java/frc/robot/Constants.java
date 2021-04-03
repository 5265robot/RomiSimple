// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {

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
        // level to keep while driving
        public static final double lightLevel = 0.50;
        // which side of the line? +1 is right, -1 is left
        public static final double lineSide = +1.0;
        // how fast?
        public static final double speed = 0.60;
        // proportional, integral and derivative
        public static final double kLineP = 5.9, kLineI = 0.0, kLineD = 0.0;
        // threshold for turning
        public static final double diffSquare = 0.007;
        // turnrate inflection
        public static final double inflection = 2.7;

    }
}
