// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.tests;

import frc.robot.tools.math.Spline;
import frc.robot.tools.math.Waypoint;

/** Add your docs here. */
public class SplineTest {
    public static void main(String[] args) {
        Spline spline = new Spline(new Waypoint[] {
                Waypoint.fromDegrees(0, 13.0, 2.0, 70.0, 0, 0, 0, 0, 0, 0),
                Waypoint.fromDegrees(1, 15.0, 4.0, -50.0, 0.5, 2, -80, 0, 0, 0),
                Waypoint.fromDegrees(5, 14.0, 6, -90, -2, 1, 0, 0, 0, 0),
                Waypoint.fromDegrees(6, 11.0, 6, 80, 0, 0, 0, 0, 0, 0),
        });
        Waypoint point = spline.interpolate(2);
        System.out.println(point.toString());
    }
}
