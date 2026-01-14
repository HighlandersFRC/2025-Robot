// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
        // public static final class Autonomous {
        //         public static final int STAGNATE_BOOST = 25;
        //         public static final int STAGNATE_THRESHOLD = 8; // Number of cycles of stagnation before ending path
        //         // lookahead distance is a function:
        //         // LOOKAHEAD = AUTONOMOUS_LOOKAHEAD_DISTANCE * velocity + MIN_LOOKAHEAD_DISTANCE
        //         // their constants
        //         public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 0.04; // Lookahead at 1m/s scaled by wanted
        //                                                                          // velocity
        //         public static final double FULL_SEND_LOOKAHEAD = 0.60;
        //         public static final double MIN_LOOKAHEAD_DISTANCE = 0.05; // Lookahead distance at 0m/s
        //         // Path follower will end if within this radius of the final point
        //         public static final double AUTONOMOUS_END_ACCURACY = 0.40;
        //         public static final double ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY = 0.05;
        //         // When calculating the point distance, will divide x and y by this constant
        //         public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1.0;
        //         // When calculating the point distance, will divide theta by this constant
        //         public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = 4 * Math.PI;
        //         // Feed Forward Multiplier
        //         public static final double FEED_FORWARD_MULTIPLIER = 0.8044;
        //         public static final double ACCURATE_FOLLOWER_FEED_FORWARD_MULTIPLIER = 1;
        //         public static final String[] paths = new String[] {
        //                         "2AlgaeCenter.polarauto",
        //                         "odometry test.polarauto",
        //                         "3PieceFeederSmart.polarauto",
        //                         "4PieceFeederGroundSmart.polarauto",
        //                         "TushPush.polarauto",
        //         };

        //         public static int getSelectedPathIndex() {
        //                 if (OI.autoChooserConnected()) {
        //                         if (OI.autoChooser.getRawButton(1)) {
        //                                 return 0;
        //                         }
        //                         if (OI.autoChooser.getRawButton(2)) {
        //                                 return 1;
        //                         }
        //                         if (OI.autoChooser.getRawButton(3)) {
        //                                 return 2;
        //                         }
        //                         if (OI.autoChooser.getRawButton(4)) {
        //                                 return 3;
        //                         }
        //                         if (OI.autoChooser.getRawButton(5)) {
        //                                 return 4;
        //                         }
        //                 } else {
        //                         return (int) Math.round(SmartDashboard.getNumber("ROBOT AUTO OVERIDE", -1));
        //                 }
        //                 return -1;
        //         }

        // }

        public static final double closedLoopSimResolution = 0.01; // seconds

        public static final double G = 9.80665;

        public static void init() {

        }

        // Physical constants (e.g. field and robot dimensions)
        public static final class Physical {

                public static final double FIELD_WIDTH = 8.052;
                public static final double FIELD_LENGTH = 17.548;
                public static final double WHEEL_DIAMETER = inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
                public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;
                public static final double WHEEL_TO_FRAME_DISTANCE = inchesToMeters(2.5); // TODO: is this different for
                                                                                          // mk5s?
                public static final double TOP_SPEED = feetToMeters(30.0);
                public static final double SIM_TOP_SPEED = 6.0; // meters per second
                public static final double MAX_ACCELERATION = feetToMeters(30.0); // TODO: actually tune the top speed
                                                                                  // and max acceleration. Add a max
                                                                                  // deceleration if needed.
                public static final double SIM_MAX_ACCELERATION = 4.0; // meters per second
                public static final double TWIST_MOI = Units.lbsToKilograms(5.98)
                                * Math.pow(Units.inchesToMeters(0.5), 2.0);// used cad to find
                public static final int TWIST_MOTOR_COUNT = 1;
                public static final double ROBOT_LENGTH = inchesToMeters(26);
                public static final double ROBOT_WIDTH = inchesToMeters(26);
                public static final double MODULE_OFFSET = inchesToMeters(2.625); // TODO: is this different for mk5s?
                public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH / 2 - WHEEL_TO_FRAME_DISTANCE,
                                ROBOT_WIDTH / 2 - WHEEL_TO_FRAME_DISTANCE);
                public static final double SIM_MAX_ANGULAR_ACCELERATION = SIM_MAX_ACCELERATION / ROBOT_RADIUS;

                public static final double GRAVITY_ACCEL_MS2 = 9.806;
        }

        // Subsystem setpoint constants
        public static final class SetPoints {

        }

        // PID constants
        public static final class PIDConstants {

        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {
                // Poses of all 16 AprilTags, {x, y, z, yaw, pitch}, in meters and radians
                public static final double[][] TAG_POSES = {};

                // Poses of cameras relative to robot, {x, y, z, rx, ry, rz}, in meters and
                // radians
                public static final double[] FRONT_CAMERA_POSE = { Constants.inchesToMeters(1.75),
                                Constants.inchesToMeters(11.625),
                                Constants.inchesToMeters(33.5), 0, -33.5, 0 };

                // Standard deviation adjustments
                public static final double STANDARD_DEVIATION_SCALAR = 1;
                public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_SCALAR = 1;
                public static final double ODOMETRY_JUMP_STANDARD_DEVIATION_DEGREE = 3;
                public static final double TAG_STANDARD_DEVIATION_DISTANCE = 2; // meters
                public static final double TAG_STANDARD_DEVIATION_FLATNESS = 5;
                // public static final double XY_STD_DEV_SCALAR = 0.1;

                // Standard deviation regressions
                /**
                 * Calculates the standard deviation scalar based on the distance from the tag.
                 *
                 * @param dist The distance from the tag.
                 * @return The standard deviation scalar.
                 */
                public static double getTagDistStdDevScalar(double dist) {
                        // double a = TAG_STANDARD_DEVIATION_FLATNESS;
                        // double b = 1 - a * Math.pow(TAG_STANDARD_DEVIATION_DISTANCE, 2);

                        return 0.0000520833 * Math.pow(dist, 4) + 0.000394571 * Math.pow(dist, 3)
                                        + 0.000440341 * Math.pow(dist, 2)
                                        + 0.0554117 * dist + 0.0298674;
                }

                /**
                 * Calculates the standard deviation scalar based on the number of detected
                 * tags.
                 *
                 * @param numTags The number of detected tags.
                 * @return The standard deviation scalar.
                 */
                public static double getNumTagStdDevScalar(int numTags) {
                        if (numTags == 0) {
                                return 99999;
                        } else if (numTags == 1) {
                                return 1;
                        } else if (numTags == 2) {
                                return 0.6;
                        } else {
                                return 0.75;
                        }
                }

                /**
                 * Calculates the standard deviation of the x-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the x-coordinate.
                 */
                public static double getTagStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.005533021491867763 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.010807566510145635)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation of the y-coordinate based on the given
                 * offsets.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation of the y-coordinate.
                 */
                public static double getTagStdDevY(double xOffset, double yOffset) {
                        return Math.max(0, 0.0055 * (xOffset * xOffset + yOffset * yOffset) - 0.01941597810542626)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the x-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the x-coordinate.
                 */
                public static double getTriStdDevX(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.004544133588821881 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.01955724864971872)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                /**
                 * Calculates the standard deviation in the y-coordinate for triangulation
                 * measurements.
                 *
                 * @param xOffset The x-coordinate offset.
                 * @param yOffset The y-coordinate offset.
                 * @return The standard deviation in the y-coordinate.
                 */
                public static double getTriStdDevY(double xOffset, double yOffset) {
                        return Math.max(0,
                                        0.002615358015002413 * (xOffset * xOffset + yOffset * yOffset)
                                                        - 0.008955462032388808)
                                        * STANDARD_DEVIATION_SCALAR;
                }

                public static double distBetweenPose(Pose3d pose1, Pose3d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }

                public static double distBetweenPose2d(Pose2d pose1, Pose2d pose2) {
                        return (Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2)
                                        + Math.pow(pose1.getY() - pose2.getY(), 2)));
                }

                public static final double DISTANCE_OFFSET = 7.0;
                public static final double CAMERA_ANGLE_OFFSET = 0.0;
                // pitch, distance
                public static final double[][] CORAL_LOOKUP_TABLE = {};

                /**
                 * Interpolates a value from a lookup table based on the given xValue.
                 * 
                 * @param xIndex The index of the x-values in the lookup table.
                 * @param yIndex The index of the y-values in the lookup table.
                 * @param xValue The x-value for which to interpolate a y-value.
                 * @return The interpolated y-value corresponding to the given x-value.
                 */
                public static double getInterpolatedValue(int xIndex, int yIndex, double xValue) {
                        int lastIndex = CORAL_LOOKUP_TABLE.length - 1;
                        if (xValue < CORAL_LOOKUP_TABLE[0][xIndex]) {
                                // If the xValue is closer than the first setpoint
                                double returnValue = CORAL_LOOKUP_TABLE[0][yIndex];
                                return returnValue;
                        } else if (xValue > CORAL_LOOKUP_TABLE[lastIndex][xIndex]) {
                                // If the xValue is farther than the last setpoint
                                double returnValue = CORAL_LOOKUP_TABLE[lastIndex][yIndex];
                                return returnValue;
                        } else {
                                for (int i = 0; i < CORAL_LOOKUP_TABLE.length; i++) {
                                        if (xValue > CORAL_LOOKUP_TABLE[i][xIndex]
                                                        && xValue < CORAL_LOOKUP_TABLE[i + 1][xIndex]) {
                                                // If the xValue is in the table of setpoints
                                                // Calculate where xValue is between setpoints
                                                double leftDif = xValue - CORAL_LOOKUP_TABLE[i][xIndex];
                                                double percent = leftDif / (CORAL_LOOKUP_TABLE[i + 1][xIndex]
                                                                - CORAL_LOOKUP_TABLE[i][xIndex]);

                                                double value1 = CORAL_LOOKUP_TABLE[i][yIndex];
                                                double value2 = CORAL_LOOKUP_TABLE[i + 1][yIndex];

                                                // Interpolate in-between values for value xValue and shooter rpm
                                                double newValue = value1 + (percent * (value2 - value1));

                                                double returnValue = newValue;
                                                return returnValue;
                                        }
                                }
                                // Should never run
                                double returnValue = CORAL_LOOKUP_TABLE[0][yIndex];
                                return returnValue;
                        }
                }

                /**
                 * Calculates shooter values (flywheel velocity and note velocity) based on the
                 * given angle.
                 *
                 * @param angle The angle of the shooter.
                 * @return An array containing the calculated flywheel velocity and note
                 *         velocity.
                 */
                public static double getCoralDistanceFromPitch(double angle) {
                        return getInterpolatedValue(0, 1, angle);
                        // return new double[] {25, getInterpolatedValue(1, 3, angle)};
                }
        }

        // Gear ratios and conversions
        public static final class Ratios {

                // drive
                // public static final double DRIVE_GEAR_RATIO = 7.03; // mk5 R1
                public static final double DRIVE_GEAR_RATIO = 7.2409; // mk5 R1 wierd?
                // public static final double DRIVE_GEAR_RATIO = 6.03; // mk5 R2
                // public static final double DRIVE_GEAR_RATIO = 5.27; // mk5 R3
                public static final double STEER_GEAR_RATIO = 26.09; // mk5

                // Testing
                public static final double TEST_MOTOR_TO_FLY_WHEEL_RATIO = 1.0;

                public class Shooter {
                        public static final double HOOD_GEAR_RATIO = 1.0; // TODO: make this the actual ratio
                        public static final double SHOOTER_GEAR_RATIO = 1.0; // TODO: make this the actual ratio
                }
        }

        // Can info such as IDs
        public static final class CANInfo {
                public static final String CANBUS_NAME = "rio";

                // drive
                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
                public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 2;
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
                public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 4;
                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
                public static final int BACK_LEFT_ANGLE_MOTOR_ID = 6;
                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
                public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 8;
                public static final int FRONT_RIGHT_MODULE_CANCODER_ID = 1;
                public static final int FRONT_LEFT_MODULE_CANCODER_ID = 2;
                public static final int BACK_LEFT_MODULE_CANCODER_ID = 3;
                public static final int BACK_RIGHT_MODULE_CANCODER_ID = 4;

                // Testing
                public static final int TEST_MOTOR_ID = 21;
                public static final int TEST_MOTOR_ID2 = 22;

                // Shooter
                public static final int SHOOTER_FLYWHEEL_MOTOR_MASTER_ID = 9;
                public static final int SHOOTER_FLYWHEEL_MOTOR_FOLLOWER_ID = 10;
                public static final int SHOOTER_HOOD_MOTOR_ID = 13;

                // Intake
                public static final int INTAKE_ROLLER_ID = 12;

                // Feeder
                public static final int FEEDER_1_MOTOR_ID = 14;
                public static final int FEEDER_2_MOTOR_ID = 15;
        }

        // Misc. controller values
        public static final class OperatorConstants {
                public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_STICK_DEADZONE = 0.03;
                public static final double RIGHT_STICK_DEADZONE = 0.05;
        }

        // Motor Specs (used for simulation)
        public static final class MotorSpecs {
                public static final class x44 {
                        public static final double X44_FREE_SPEED_RPM = 7530;
                        public static final double X44_STALL_TORQUE_NM = 4.05;
                        public static final double X44_STALL_CURRENT_A = 275;
                        public static final double X44_FREE_CURRENT_A = 1.4;
                        public static final double X44_NOMINAL_VOLTAGE_V = 12;

                        public static DCMotor getX44Gearbox(int numMotors) {
                                return new DCMotor(X44_NOMINAL_VOLTAGE_V, X44_STALL_TORQUE_NM, X44_STALL_CURRENT_A,
                                                X44_FREE_CURRENT_A, Units.rotationsPerMinuteToRadiansPerSecond(
                                                                X44_FREE_SPEED_RPM),
                                                numMotors);
                        }
                }
        }

        /**
         * Converts inches to meters.
         *
         * @param inches The length in inches to be converted.
         * @return The equivalent length in meters.
         */
        public static double inchesToMeters(double inches) {
                return inches / 39.37;
        }

        public static double metersToInches(double meters) {
                return meters * 39.37;
        }

        /**
         * Converts feet to meters.
         *
         * @param inches The length in feet to be converted.
         * @return The equivalent length in meters.
         */
        public static double feetToMeters(double feet) {
                return feet / 3.281;
        }

        /**
         * Calculates the Euclidean distance between two points in a 2D plane.
         *
         * @param x1 The x-coordinate of the first point.
         * @param y1 The y-coordinate of the first point.
         * @param x2 The x-coordinate of the second point.
         * @param y2 The y-coordinate of the second point.
         * @return The Euclidean distance between the two points.
         */
        public static double getDistance(double x1, double y1, double x2, double y2) {
                return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
        }

        public static double getAngleToPoint(double x1, double y1, double x2, double y2) {
                double deltaX = x2 - x1;
                double deltaY = y2 - y1;

                double angleInRadians = Math.atan2(deltaY, deltaX);

                double angleInDegrees = Math.toDegrees(angleInRadians);

                double standardizeAngleDegrees = standardizeAngleDegrees(angleInDegrees);

                return 180 + standardizeAngleDegrees;
        }

        /**
         * Converts a quantity in rotations to radians.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double rotationsToRadians(double rotations) {
                return rotations * 2 * Math.PI;
        }

        /**
         * Converts a quantity in degrees to rotations.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in rotations.
         */
        public static double degreesToRotations(double degrees) {
                return degrees / 360;
        }

        /**
         * Converts a quantity in rotations to degrees.
         *
         * @param rotations The quantity in rotations to be converted.
         * @return The equivalent quantity in degrees.
         */
        public static double rotationsToDegrees(double rotations) {
                return rotations * 360;
        }

        /**
         * Converts a quantity in degrees to radians.
         *
         * @param rotations The quantity in degrees to be converted.
         * @return The equivalent quantity in radians.
         */
        public static double degreesToRadians(double degrees) {
                return degrees * Math.PI / 180;
        }

        /**
         * Standardizes an angle to be within the range [0, 360) degrees.
         *
         * @param angleDegrees The input angle in degrees.
         * @return The standardized angle within the range [0, 360) degrees.
         */
        public static double standardizeAngleDegrees(double angleDegrees) {
                return ((angleDegrees % 360) + 360) % 360;
        }

        /**
         * Calculates the x-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The x-component of the unit vector.
         */
        public static double angleToUnitVectorI(double angle) {
                return (Math.cos(angle));
        }

        /**
         * Calculates the y-component of a unit vector given an angle in radians.
         *
         * @param angle The angle in radians.
         * @return The y-component of the unit vector.
         */
        public static double angleToUnitVectorJ(double angle) {
                return (Math.sin(angle));
        }

        /**
         * Converts revolutions per minute (RPM) to revolutions per second (RPS).
         *
         * @param RPM The value in revolutions per minute (RPM) to be converted.
         * @return The equivalent value in revolutions per second (RPS).
         */
        public static double RPMToRPS(double RPM) {
                return RPM / 60;
        }

        /**
         * Converts revolutions per second (RPS) to revolutions per minute (RPM).
         *
         * @param RPM The value in revolutions per second (RPS) to be converted.
         * @return The equivalent value in revolutions per minute (RPM).
         */
        public static double RPSToRPM(double RPS) {
                return RPS * 60;
        }

        /**
         * Standardizes an angle to be within the range [otherAngle - pi, otherAngle +
         * pi) radians.
         *
         * @param angle      The input angle in radians.
         * @param otherAngle The reference angle in radians.
         * @return The standardized angle within the range [otherAngle - pi, otherAngle
         *         +
         *         pi) radians.
         */
        public static double standardizeAngleToOther(double angle, double otherAngle) {
                double delta = angle - otherAngle;
                delta = Math.IEEEremainder(delta, 2 * Math.PI); // gives value in [-π, π]
                return otherAngle + delta;
        }

        /**
         * Standardizes an angle to be within the range [otherAngle - 180, otherAngle +
         * 180) degrees.
         *
         * @param angle      The input angle in degrees.
         * @param otherAngle The reference angle in degrees.
         * @return The standardized angle within the range [otherAngle - 180, otherAngle
         *         +
         *         180) degrees.
         */
        public static double standardizeAngleToOtherDegrees(double angle, double otherAngle) {
                return Math.toDegrees(standardizeAngleToOther(degreesToRadians(angle), degreesToRadians(otherAngle)));
        }
}
