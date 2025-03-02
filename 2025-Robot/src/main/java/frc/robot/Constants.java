// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.tools.math.Vector;

public final class Constants {
        public static final class Autonomous {
                // lookahead distance is a function:
                // LOOKAHEAD = AUTONOMOUS_LOOKAHEAD_DISTANCE * velocity + MIN_LOOKAHEAD_DISTANCE
                // their constants
                public static final double AUTONOMOUS_LOOKAHEAD_DISTANCE = 0.10; // Lookahead at 1m/s scaled by wanted
                                                                                 // velocity
                public static final double FULL_SEND_LOOKAHEAD = 0.60;
                public static final double MIN_LOOKAHEAD_DISTANCE = 0.01; // Lookahead distance at 0m/s
                // Path follower will end if within this radius of the final point
                public static final double AUTONOMOUS_END_ACCURACY = 0.25;
                public static final double ACCURATE_FOLLOWER_AUTONOMOUS_END_ACCURACY = 0.05;
                // When calculating the point distance, will divide x and y by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS = 1.0;
                // When calculating the point distance, will divide theta by this constant
                public static final double AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS = Math.PI;
                // Feed Forward Multiplier
                public static final double FEED_FORWARD_MULTIPLIER = 0.5;
                public static final double ACCURATE_FOLLOWER_FEED_FORWARD_MULTIPLIER = 1;
                public static final String[] paths = new String[] {
                                "Test.polarauto",
                                "3piece.polarauto",
                                "3PieceFeeder.polarauto",
                                "3PieceFeederSmart.polarauto",
                                "2+1PieceFeeder.polarauto",
                };

                public static int getSelectedPathIndex() {
                        if (!OI.autoChooser.getRawButton(7)) {
                                if (OI.autoChooser.getRawButton(1)) {
                                        return 0;
                                }
                                if (OI.autoChooser.getRawButton(2)) {
                                        return 1;
                                }
                                if (OI.autoChooser.getRawButton(3)) {
                                        return 2;
                                }
                                if (OI.autoChooser.getRawButton(4)) {
                                        return 3;
                                }
                                if (OI.autoChooser.getRawButton(5)) {
                                        return 4;
                                }
                        } else {
                                if (OI.autoChooser.getRawButton(1)) {
                                        return 5;
                                }
                                if (OI.autoChooser.getRawButton(2)) {
                                        return 6;
                                }
                                if (OI.autoChooser.getRawButton(3)) {
                                        return 7;
                                }
                                if (OI.autoChooser.getRawButton(4)) {
                                        return 8;
                                }
                                if (OI.autoChooser.getRawButton(5)) {
                                        return 9;
                                }
                        }
                        return -1;
                }

        }

        public static void periodic() {
                int index = Autonomous.getSelectedPathIndex();
                if (index == -1 || index > Constants.Autonomous.paths.length) {
                        Logger.recordOutput("Selected Auto", "Do Nothing");
                } else {
                        Logger.recordOutput("Selected Auto", Autonomous.paths[index]);
                }
        }

        public static void init() {

                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint1);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint2);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint3);
                // ///////
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint4);
                // // Only have these 4 now
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint5);
                // // The rest are 0, 0
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint6);
                // ///////
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint7);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint8);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint9);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint10);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint11);
                // Constants.Physical.redCoralScoringPositions.add(Constants.Physical.redSetpoint12);

                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint1);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint2);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint3);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint4);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint5);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint6);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint7);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint8);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint9);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint10);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint11);
                // Constants.Physical.blueCoralScoringPositions.add(Constants.Physical.blueSetpoint12);

                // for (int i = 0; i < Constants.Vision.redSideReefTags.length; i++) {
                // Vector tagVector = new Vector(Constants.Vision.redSideReefTags[i][0],
                // Constants.Vision.redSideReefTags[i][1]);
                // Vector offsetXVector = new Vector(
                // Constants.Physical.CORAL_PLACEMENT_X *
                // Math.cos(Constants.Vision.redSideReefTags[i][3]),
                // Constants.Physical.CORAL_PLACEMENT_X *
                // Math.sin(Constants.Vision.redSideReefTags[i][3]));
                // Vector offsetYVector = new Vector(
                // Constants.Physical.CORAL_PLACEMENT_Y *
                // Math.sin(Constants.Vision.redSideReefTags[i][3]),
                // Constants.Physical.CORAL_PLACEMENT_Y *
                // Math.cos(Constants.Vision.redSideReefTags[i][3]));
                // Vector leftVector = tagVector.add(offsetXVector.add(offsetYVector));
                // Vector rightVector = tagVector.add(offsetXVector.subtract(offsetYVector));
                // Constants.Physical.redCoralScoringPositions
                // .add(new Pose2d(new Translation2d(leftVector.getI(), leftVector.getJ()),
                // new Rotation2d(Constants.Vision.redSideReefTags[i][3] + Math.PI)));
                // Constants.Physical.redCoralScoringPositions
                // .add(new Pose2d(new Translation2d(rightVector.getI(), rightVector.getJ()),
                // new Rotation2d(Constants.Vision.redSideReefTags[i][3] + Math.PI)));
                // }
                // for (int i = 0; i < Constants.Vision.blueSideReefTags.length; i++) {
                // Vector tagVector = new Vector(Constants.Vision.blueSideReefTags[i][0],
                // Constants.Vision.blueSideReefTags[i][1]);
                // Vector offsetXVector = new Vector(
                // Constants.Physical.CORAL_PLACEMENT_X *
                // Math.cos(Constants.Vision.blueSideReefTags[i][3]),
                // Constants.Physical.CORAL_PLACEMENT_X *
                // Math.sin(Constants.Vision.blueSideReefTags[i][3]));
                // Vector offsetYVector = new Vector(
                // Constants.Physical.CORAL_PLACEMENT_Y *
                // Math.sin(Constants.Vision.blueSideReefTags[i][3]),
                // Constants.Physical.CORAL_PLACEMENT_Y *
                // Math.cos(Constants.Vision.blueSideReefTags[i][3]));
                // Vector leftVector = tagVector.add(offsetXVector.add(offsetYVector));
                // Vector rightVector = tagVector.add(offsetXVector.subtract(offsetYVector));
                // Constants.Physical.blueCoralScoringPositions
                // .add(new Pose2d(new Translation2d(leftVector.getI(), leftVector.getJ()),
                // new Rotation2d(Constants.Vision.blueSideReefTags[i][3] + Math.PI)));
                // Constants.Physical.blueCoralScoringPositions
                // .add(new Pose2d(new Translation2d(rightVector.getI(), rightVector.getJ()),
                // new Rotation2d(Constants.Vision.blueSideReefTags[i][3] + Math.PI)));
                // }

                // Logger.recordOutput("red side scoring",
                // Constants.Physical.redCoralScoringPositions.toString());
                // Logger.recordOutput("blue side scoring",
                // Constants.Physical.blueCoralScoringPositions.toString());
                System.out.println("blue algae front positions: "
                                + Constants.Reef.algaeBlueFrontPlacingPositions.toString());
                System.out.println("red algae front positions: "
                                + Constants.Reef.algaeRedFrontPlacingPositions.toString());
                System.out.println("blue algae back positions: "
                                + Constants.Reef.algaeBlueBackPlacingPositions.toString());
                System.out.println(
                                "red algae back positions: " + Constants.Reef.algaeRedBackPlacingPositions.toString());

                System.out.println("blue positions: " + Constants.Reef.blueFrontPlacingPositions.toString());
                System.out.println("red positions: " + Constants.Reef.redFrontPlacingPositions.toString());
                System.out.println("blue back positions: " + Constants.Reef.blueBackPlacingPositions.toString());
                System.out.println("red back positions: " + Constants.Reef.redBackPlacingPositions.toString());

                System.out.println("l4 blue positions: " + Constants.Reef.l4BlueFrontPlacingPositions.toString());
                System.out.println("l4 red positions: " + Constants.Reef.l4RedFrontPlacingPositions.toString());
                System.out.println("l4 blue back positions: " + Constants.Reef.l4BlueBackPlacingPositions.toString());
                System.out.println("l4 red back positions: " + Constants.Reef.l4RedBackPlacingPositions.toString());

                System.out.println("l3 blue positions: " + Constants.Reef.l3BlueFrontPlacingPositions.toString());
                System.out.println("l3 red positions: " + Constants.Reef.l3RedFrontPlacingPositions.toString());
                System.out.println("l3 blue back positions: " + Constants.Reef.l3BlueBackPlacingPositions.toString());
                System.out.println("l3 red back positions: " + Constants.Reef.l3RedBackPlacingPositions.toString());
        }

        public static class Reef {
                public static final Translation2d centerBlue = new Translation2d(inchesToMeters(176.746),
                                inchesToMeters(158.501));
                public static final Translation2d centerRed = new Translation2d(
                                Constants.Physical.FIELD_LENGTH - inchesToMeters(176.746),
                                inchesToMeters(158.501));
                public static final double faceToZoneLine = inchesToMeters(12); // Side of the reef to the inside of the
                                                                                // reef
                                                                                // zone line

                public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in
                                                                          // clockwise
                                                                          // order
                                                                          // public static final List<Map<ReefHeight,
                                                                          // Pose3d>>
                                                                          // blueBranchPositions = new
                                                                          // ArrayList<>(); // Starting at the right
                                                                          // // branch facing the driver
                                                                          // // station in clockwise
                                                                          // public static final List<Map<ReefHeight,
                                                                          // Pose3d>>
                                                                          // redBranchPositions = new
                                                                          // ArrayList<>(); // Starting at the right

                public static final double RED_LEFT_FEEDER_X = 16.404;
                public static final double RED_LEFT_FEEDER_Y = 0.990;
                public static final double RED_LEFT_FEEDER_THETA = Math.toRadians(126.0);

                public static final Pose2d RED_LEFT_FEEDER = new Pose2d(RED_LEFT_FEEDER_X, RED_LEFT_FEEDER_Y,
                                new Rotation2d(RED_LEFT_FEEDER_THETA));
                public static final Pose2d RED_RIGHT_FEEDER = new Pose2d(
                                RED_LEFT_FEEDER_X, Physical.FIELD_WIDTH - RED_LEFT_FEEDER_Y,
                                new Rotation2d(Math.toRadians(234.0)));

                public static final Pose2d BLUE_RIGHT_FEEDER = new Pose2d(
                                Physical.FIELD_LENGTH - RED_LEFT_FEEDER_X, RED_LEFT_FEEDER_Y,
                                new Rotation2d(Math.toRadians(54.0)));
                public static final Pose2d BLUE_LEFT_FEEDER = new Pose2d(
                                Physical.FIELD_LENGTH - RED_LEFT_FEEDER_X, Physical.FIELD_WIDTH - RED_LEFT_FEEDER_Y,
                                new Rotation2d(Math.toRadians(-54.0)));

                public static final double PROCESSOR_Y_OFFSET_M = inchesToMeters(93); // 20?
                public static final double NET_X_OFFSET_M = inchesToMeters(93); // 30?

                public static final Translation2d processorBlueFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_Y_OFFSET_M); // TODO:
                // actually
                // measure
                // these
                public static final Rotation2d processorBlueFrontPlacingRotation = new Rotation2d(
                                degreesToRadians(270));

                public static final Translation2d processorRedFrontPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorRedFrontPlacingRotation = new Rotation2d(degreesToRadians(90));

                public static final Translation2d processorBlueBackPlacingTranslation = new Translation2d(
                                inchesToMeters(238.79),
                                PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorBlueBackPlacingRotation = new Rotation2d(degreesToRadians(90));

                public static final Translation2d processorRedBackPlacingTranslation = new Translation2d(
                                inchesToMeters(452.40),
                                inchesToMeters(316.21) - PROCESSOR_Y_OFFSET_M);
                public static final Rotation2d processorRedBackPlacingRotation = new Rotation2d(degreesToRadians(270));

                public static final Pose2d processorBlueFrontPlacingPosition = new Pose2d(
                                processorBlueFrontPlacingTranslation,
                                processorBlueFrontPlacingRotation);
                public static final Pose2d processorRedFrontPlacingPosition = new Pose2d(
                                processorRedFrontPlacingTranslation,
                                processorRedFrontPlacingRotation);
                public static final Pose2d processorBlueBackPlacingPosition = new Pose2d(
                                processorBlueBackPlacingTranslation,
                                processorBlueBackPlacingRotation);
                public static final Pose2d processorRedBackPlacingPosition = new Pose2d(
                                processorRedBackPlacingTranslation,
                                processorRedBackPlacingRotation);

                public static final double netBlueXM = (Constants.Physical.FIELD_LENGTH / 2) - NET_X_OFFSET_M; // TODO:
                                                                                                               // measure
                                                                                                               // these
                                                                                                               // ones
                                                                                                               // too
                public static final double netRedXM = (Constants.Physical.FIELD_LENGTH / 2) + NET_X_OFFSET_M;

                public static final double netBlueFrontThetaR = degreesToRadians(0.0);
                public static final double netRedFrontThetaR = degreesToRadians(180.0);
                public static final double netBlueBackThetaR = degreesToRadians(180.0);
                public static final double netRedBackThetaR = degreesToRadians(0.0);

                public static final List<Pose2d> blueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> redFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> blueBackPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> redBackPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> blueFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> redFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> blueBackPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> redBackPlacingPositionsMore = new ArrayList<>();

                public static final List<Pose2d> l4BlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l4RedFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l4BlueBackPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l4RedBackPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> l3BlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l3RedFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l3BlueBackPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> l3RedBackPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> algaeBlueFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> algaeRedFrontPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> algaeBlueBackPlacingPositions = new ArrayList<>();
                public static final List<Pose2d> algaeRedBackPlacingPositions = new ArrayList<>();

                public static final List<Pose2d> algaeBlueFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> algaeRedFrontPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> algaeBlueBackPlacingPositionsMore = new ArrayList<>();
                public static final List<Pose2d> algaeRedBackPlacingPositionsMore = new ArrayList<>();

                static {
                        calculateReefPoints();
                }

                public static void calculateReefPoints() {
                        System.out.println("recalculating points");
                        blueFrontPlacingPositions.clear();
                        redFrontPlacingPositions.clear();
                        blueBackPlacingPositions.clear();
                        redBackPlacingPositions.clear();
                        blueFrontPlacingPositionsMore.clear();
                        redFrontPlacingPositionsMore.clear();
                        blueBackPlacingPositionsMore.clear();
                        redBackPlacingPositionsMore.clear();
                        l4BlueFrontPlacingPositions.clear();
                        l4RedFrontPlacingPositions.clear();
                        l4BlueBackPlacingPositions.clear();
                        l4RedBackPlacingPositions.clear();
                        l3BlueFrontPlacingPositions.clear();
                        l3RedFrontPlacingPositions.clear();
                        l3BlueBackPlacingPositions.clear();
                        l3RedBackPlacingPositions.clear();
                        algaeBlueFrontPlacingPositions.clear();
                        algaeRedFrontPlacingPositions.clear();
                        algaeBlueBackPlacingPositions.clear();
                        algaeRedBackPlacingPositions.clear();
                        algaeBlueFrontPlacingPositionsMore.clear();
                        algaeRedFrontPlacingPositionsMore.clear();
                        algaeBlueBackPlacingPositionsMore.clear();
                        algaeRedBackPlacingPositionsMore.clear();
                        centerFaces[0] = new Pose2d(
                                        inchesToMeters(144.003),
                                        inchesToMeters(158.500),
                                        Rotation2d.fromDegrees(180));
                        centerFaces[1] = new Pose2d(
                                        inchesToMeters(160.373),
                                        inchesToMeters(186.857),
                                        Rotation2d.fromDegrees(120));
                        centerFaces[2] = new Pose2d(
                                        inchesToMeters(193.116),
                                        inchesToMeters(186.858),
                                        Rotation2d.fromDegrees(60));
                        centerFaces[3] = new Pose2d(
                                        inchesToMeters(209.489),
                                        inchesToMeters(158.502),
                                        Rotation2d.fromDegrees(0));
                        centerFaces[4] = new Pose2d(
                                        inchesToMeters(193.118),
                                        inchesToMeters(130.145),
                                        Rotation2d.fromDegrees(-60));
                        centerFaces[5] = new Pose2d(
                                        inchesToMeters(160.375),
                                        inchesToMeters(130.144),
                                        Rotation2d.fromDegrees(-120));

                        for (int face = 0; face < 6; face++) {
                                Pose2d l2FrontRight = new Pose2d();
                                Pose2d l2FrontLeft = new Pose2d();
                                Pose2d frontRightMore = new Pose2d();
                                Pose2d frontLeftMore = new Pose2d();
                                Pose2d backRightMore = new Pose2d();
                                Pose2d backLeftMore = new Pose2d();
                                Pose2d l2BackRight = new Pose2d();
                                Pose2d l2BackLeft = new Pose2d();
                                Pose2d l3FrontRight = new Pose2d();
                                Pose2d l3FrontLeft = new Pose2d();
                                Pose2d l3BackRight = new Pose2d();
                                Pose2d l3BackLeft = new Pose2d();
                                Pose2d l4FrontRight = new Pose2d();
                                Pose2d l4FrontLeft = new Pose2d();
                                Pose2d l4BackRight = new Pose2d();
                                Pose2d l4BackLeft = new Pose2d();
                                Pose2d algaeFront = new Pose2d();
                                Pose2d algaeBack = new Pose2d();
                                Pose2d algaeFrontMore = new Pose2d();
                                Pose2d algaeBackMore = new Pose2d();
                                Pose2d poseDirection = new Pose2d(centerBlue,
                                                Rotation2d.fromDegrees(180 - (60 * face)));
                                double adjustX = inchesToMeters(30.738);
                                double adjustY = inchesToMeters(6.469);
                                double adjustXMore = inchesToMeters(50.738);
                                double adjustYMore = inchesToMeters(6.469);
                                double adjustAlgaeX = inchesToMeters(35.738);
                                double adjustAlgaeY = inchesToMeters(0.0);
                                double adjustAlgaeMoreX = inchesToMeters(16.738);
                                double adjustAlgaeMoreY = inchesToMeters(0.0);

                                algaeFront = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                algaeBack = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_BACK_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_BACK_ALGAE,
                                                                                                                new Rotation2d(Math.PI))) // TODO
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustAlgaeX,
                                                                                                adjustAlgaeY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_BACK_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_BACK_ALGAE,
                                                                                                                new Rotation2d(Math.PI))) // TODO
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                algaeFrontMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_FRONT_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_FRONT_ALGAE,
                                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));

                                algaeBackMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_BACK_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_BACK_ALGAE,
                                                                                                                new Rotation2d(Math.PI))) // TODO:
                                                                                                                                          // why
                                                                                                                                          // is
                                                                                                                                          // this
                                                                                                                                          // pi
                                                                                                                                          // and
                                                                                                                                          // not
                                                                                                                                          // 0
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(
                                                                                                new Transform2d(adjustAlgaeMoreX,
                                                                                                                adjustAlgaeMoreY,
                                                                                                                new Rotation2d()))
                                                                                .transformBy(
                                                                                                new Transform2d(Physical.INTAKE_X_OFFSET_BACK_ALGAE,
                                                                                                                Physical.INTAKE_Y_OFFSET_BACK_ALGAE,
                                                                                                                new Rotation2d(Math.PI))) // TODO
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));

                                frontRightMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                backRightMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));

                                l2FrontRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l2BackRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                l4FrontRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l4BackRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                l3FrontRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l3BackRight = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                l2FrontLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l2BackLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                frontLeftMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                backLeftMore = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(
                                                                                                adjustXMore,
                                                                                                -adjustYMore,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.INTAKE_X_OFFSET_BACK,
                                                                                                Physical.INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                l4FrontLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l4BackLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L4_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L4_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                l3FrontLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_FRONT,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_FRONT,
                                                                                                new Rotation2d(Math.PI)))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians() - Math.PI));
                                l3BackLeft = new Pose2d(
                                                new Translation2d(
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getX(),
                                                                poseDirection
                                                                                .transformBy(new Transform2d(adjustX,
                                                                                                -adjustY,
                                                                                                new Rotation2d()))
                                                                                .transformBy(new Transform2d(
                                                                                                Physical.L3_INTAKE_X_OFFSET_BACK,
                                                                                                Physical.L3_INTAKE_Y_OFFSET_BACK,
                                                                                                new Rotation2d()))
                                                                                .getY()),
                                                new Rotation2d(
                                                                poseDirection.getRotation().getRadians()));
                                blueFrontPlacingPositions.add(l2FrontRight);
                                blueFrontPlacingPositions.add(l2FrontLeft);
                                blueBackPlacingPositions.add(l2BackRight);
                                blueBackPlacingPositions.add(l2BackLeft);
                                blueFrontPlacingPositionsMore.add(frontRightMore);
                                blueFrontPlacingPositionsMore.add(frontLeftMore);
                                blueBackPlacingPositionsMore.add(backRightMore);
                                blueBackPlacingPositionsMore.add(backLeftMore);
                                l4BlueFrontPlacingPositions.add(l4FrontRight);
                                l4BlueFrontPlacingPositions.add(l4FrontLeft);
                                l4BlueBackPlacingPositions.add(l4BackRight);
                                l4BlueBackPlacingPositions.add(l4BackLeft);
                                l3BlueFrontPlacingPositions.add(l3FrontRight);
                                l3BlueFrontPlacingPositions.add(l3FrontLeft);
                                l3BlueBackPlacingPositions.add(l3BackRight);
                                l3BlueBackPlacingPositions.add(l3BackLeft);
                                algaeBlueFrontPlacingPositions.add(algaeFront);
                                algaeBlueBackPlacingPositions.add(algaeBack);
                                algaeBlueFrontPlacingPositionsMore.add(algaeFrontMore);
                                algaeBlueBackPlacingPositionsMore.add(algaeBackMore);
                        }

                        for (Pose2d bluePose : algaeBlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : algaeBlueBackPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedBackPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : algaeBlueFrontPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedFrontPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : algaeBlueBackPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                algaeRedBackPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : blueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : blueBackPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() - Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redBackPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : blueFrontPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redFrontPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : blueBackPlacingPositionsMore) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() - Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                redBackPlacingPositionsMore.add(redPose);
                        }

                        for (Pose2d bluePose : l4BlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l4RedFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : l4BlueBackPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() - Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l4RedBackPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : l3BlueFrontPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() + Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l3RedFrontPlacingPositions.add(redPose);
                        }

                        for (Pose2d bluePose : l3BlueBackPlacingPositions) {
                                Pose2d redPose = new Pose2d();
                                Translation2d mirroredTranslation = new Translation2d(
                                                Constants.Physical.FIELD_LENGTH - bluePose.getX(),
                                                Constants.Physical.FIELD_WIDTH - bluePose.getY());
                                Rotation2d mirroredRotation = new Rotation2d(
                                                bluePose.getRotation().getRadians() - Math.PI);
                                redPose = new Pose2d(mirroredTranslation, mirroredRotation);
                                l3RedBackPlacingPositions.add(redPose);
                        }
                }
        }

        // Physical constants (e.g. field and robot dimensions)
        public static final class Physical {
                public static final double FIELD_WIDTH = 8.052;
                public static final double FIELD_LENGTH = 17.548;
                public static final double WHEEL_DIAMETER = inchesToMeters(4);
                public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
                public static final double WHEEL_ROTATION_PER_METER = 1 / WHEEL_CIRCUMFERENCE;
                public static final double WHEEL_TO_FRAME_DISTANCE = inchesToMeters(2.5);
                public static final double TOP_SPEED = feetToMeters(30.0);

                public static final double ROBOT_LENGTH = inchesToMeters(26);
                public static final double ROBOT_WIDTH = inchesToMeters(26);
                public static final double MODULE_OFFSET = inchesToMeters(2.5);
                public static final double ROBOT_RADIUS = Math.hypot(ROBOT_LENGTH / 2 - WHEEL_TO_FRAME_DISTANCE,
                                ROBOT_WIDTH / 2 - WHEEL_TO_FRAME_DISTANCE);

                // public static final double INTAKE_X_OFFSET_FRONT = inchesToMeters(22.5);
                // public static final double INTAKE_Y_OFFSET_FRONT = inchesToMeters(4.3);
                // public static final double INTAKE_X_OFFSET_BACK = inchesToMeters(22.5);
                // public static final double INTAKE_Y_OFFSET_BACK = inchesToMeters(-4.3);

                public static double INTAKE_X_OFFSET_FRONT = inchesToMeters(26);
                // public static final double INTAKE_X_OFFSET_FRONT = inchesToMeters(24.5);
                public static double INTAKE_Y_OFFSET_FRONT = inchesToMeters(3.25);
                public static double INTAKE_X_OFFSET_BACK = inchesToMeters(27);
                // public static final double INTAKE_X_OFFSET_BACK = inchesToMeters(25.5);
                public static double INTAKE_Y_OFFSET_BACK = inchesToMeters(-3.25);

                public static double INTAKE_X_OFFSET_FRONT_ALGAE = inchesToMeters(25.5 + 5.0);
                public static double INTAKE_Y_OFFSET_FRONT_ALGAE = inchesToMeters(3.8);
                public static double INTAKE_X_OFFSET_BACK_ALGAE = inchesToMeters(26.5 + 5.0);
                public static double INTAKE_Y_OFFSET_BACK_ALGAE = inchesToMeters(-3.8);

                public static final double L3_INTAKE_X_OFFSET_FRONT = inchesToMeters(29.5);
                public static final double L3_INTAKE_Y_OFFSET_FRONT = inchesToMeters(4.2);

                public static final double L3_INTAKE_X_OFFSET_BACK = inchesToMeters(29.5);
                public static final double L3_INTAKE_Y_OFFSET_BACK = inchesToMeters(-4.2);

                public static double L4_INTAKE_X_OFFSET_FRONT = inchesToMeters(26.6);
                // public static final double L4_INTAKE_X_OFFSET_FRONT = inchesToMeters(26.3);
                public static double L4_INTAKE_Y_OFFSET_FRONT = inchesToMeters(3.2);
                public static double L4_INTAKE_X_OFFSET_BACK = inchesToMeters(27.1);
                // public static final double L4_INTAKE_X_OFFSET_BACK = inchesToMeters(27.1);
                public static double L4_INTAKE_Y_OFFSET_BACK = inchesToMeters(-2.7);

                public static final double GRAVITY_ACCEL_MS2 = 9.806;

                // public static final Translation2d redTranslationSetpoint1 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint1 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint1 = new Pose2d(redTranslationSetpoint1,
                // redRotationSetpoint1);

                // public static final Translation2d redTranslationSetpoint2 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint2 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint2 = new Pose2d(redTranslationSetpoint2,
                // redRotationSetpoint2);

                // public static final Translation2d redTranslationSetpoint3 = new
                // Translation2d(13.70, 2.80);
                // public static final Rotation2d redRotationSetpoint3 = new Rotation2d(2.094);
                // public static final Pose2d redSetpoint3 = new Pose2d(redTranslationSetpoint3,
                // redRotationSetpoint3);

                // public static final Translation2d redTranslationSetpoint4 = new
                // Translation2d(14.00, 3.00);
                // public static final Rotation2d redRotationSetpoint4 = new Rotation2d(2.094);
                // public static final Pose2d redSetpoint4 = new Pose2d(redTranslationSetpoint4,
                // redRotationSetpoint4);

                // public static final Translation2d redTranslationSetpoint5 = new
                // Translation2d(0, 0);
                // public static final Rotation2d redRotationSetpoint5 = new Rotation2d(0);
                // public static final Pose2d redSetpoint5 = new Pose2d(redTranslationSetpoint5,
                // redRotationSetpoint5);

                // public static final Translation2d redTranslationSetpoint6 = new
                // Translation2d(0, 0);
                // public static final Rotation2d redRotationSetpoint6 = new Rotation2d(0);
                // public static final Pose2d redSetpoint6 = new Pose2d(redTranslationSetpoint6,
                // redRotationSetpoint6);

                // public static final Translation2d redTranslationSetpoint7 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint7 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint7 = new Pose2d(redTranslationSetpoint7,
                // redRotationSetpoint7);

                // public static final Translation2d redTranslationSetpoint8 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint8 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint8 = new Pose2d(redTranslationSetpoint8,
                // redRotationSetpoint8);

                // public static final Translation2d redTranslationSetpoint9 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint9 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint9 = new Pose2d(redTranslationSetpoint9,
                // redRotationSetpoint9);

                // public static final Translation2d redTranslationSetpoint10 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint10 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint10 = new
                // Pose2d(redTranslationSetpoint10, redRotationSetpoint10);

                // public static final Translation2d redTranslationSetpoint11 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint11 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint11 = new
                // Pose2d(redTranslationSetpoint11, redRotationSetpoint11);

                // public static final Translation2d redTranslationSetpoint12 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d redRotationSetpoint12 = new Rotation2d(0.0);
                // public static final Pose2d redSetpoint12 = new
                // Pose2d(redTranslationSetpoint12, redRotationSetpoint12);

                // public static final Translation2d blueTranslationSetpoint1 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint1 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint1 = new
                // Pose2d(blueTranslationSetpoint1, blueRotationSetpoint1);

                // public static final Translation2d blueTranslationSetpoint2 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint2 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint2 = new
                // Pose2d(blueTranslationSetpoint2, blueRotationSetpoint2);

                // public static final Translation2d blueTranslationSetpoint3 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint3 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint3 = new
                // Pose2d(blueTranslationSetpoint3, blueRotationSetpoint3);

                // public static final Translation2d blueTranslationSetpoint4 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint4 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint4 = new
                // Pose2d(blueTranslationSetpoint4, blueRotationSetpoint4);

                // public static final Translation2d blueTranslationSetpoint5 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint5 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint5 = new
                // Pose2d(blueTranslationSetpoint5, blueRotationSetpoint5);

                // public static final Translation2d blueTranslationSetpoint6 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint6 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint6 = new
                // Pose2d(blueTranslationSetpoint6, blueRotationSetpoint6);

                // public static final Translation2d blueTranslationSetpoint7 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint7 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint7 = new
                // Pose2d(blueTranslationSetpoint7, blueRotationSetpoint7);

                // public static final Translation2d blueTranslationSetpoint8 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint8 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint8 = new
                // Pose2d(blueTranslationSetpoint8, blueRotationSetpoint8);

                // public static final Translation2d blueTranslationSetpoint9 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint9 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint9 = new
                // Pose2d(blueTranslationSetpoint9, blueRotationSetpoint9);

                // public static final Translation2d blueTranslationSetpoint10 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint10 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint10 = new
                // Pose2d(blueTranslationSetpoint10, blueRotationSetpoint10);

                // public static final Translation2d blueTranslationSetpoint11 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint11 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint11 = new
                // Pose2d(blueTranslationSetpoint11, blueRotationSetpoint11);

                // public static final Translation2d blueTranslationSetpoint12 = new
                // Translation2d(0.0, 0.0);
                // public static final Rotation2d blueRotationSetpoint12 = new Rotation2d(0.0);
                // public static final Pose2d blueSetpoint12 = new
                // Pose2d(blueTranslationSetpoint12, blueRotationSetpoint12);

                // // x, y, theta
                // public static final ArrayList<Pose2d> redCoralScoringPositions = new
                // ArrayList<Pose2d>();
                // public static final ArrayList<Pose2d> blueCoralScoringPositions = new
                // ArrayList<Pose2d>();
        }

        // Subsystem setpoint constants
        public static final class SetPoints {
                public static final double ELEVATOR_BOTTOM_POSITION_M = 0.0;
                public static final double ELEVATOR_MID_POSITION_M = inchesToMeters(26.0); // L2 after placement
                public static final double ELEVATOR_TOP_POSITION_M = inchesToMeters(43.0);
                public static final double ELEVATOR_L1_POSITION_M = inchesToMeters(10.0);
                public static final double ELEVATOR_L2_POSITION_M = inchesToMeters(18);
                public static final double ELEVATOR_AUTO_L2_POSITION_M = inchesToMeters(17);
                public static final double ELEVATOR_AUTO_L3_POSITION_M = inchesToMeters(33);
                // public static final double ELEVATOR_AUTO_L3_POSITION_M = inchesToMeters(25);
                public static final double ELEVATOR_AUTO_SCORE_L3_POSITION_M = inchesToMeters(20);
                public static final double ELEVATOR_AUTO_L4_POSITION_M = inchesToMeters(64.0);
                public static final double ELEVATOR_L3_POSITION_M = inchesToMeters(31);
                public static final double ELEVATOR_L4_POSITION_M = inchesToMeters(64.0);
                public static final double ELEVATOR_ALGAE_POSITION_M = inchesToMeters(8.0);
                public static final double ELEVATOR_GROUND_CORAL_POSITION_M = inchesToMeters(7.0);
                public static final double ELEVATOR_GROUND_ALGAE_POSITION_M = inchesToMeters(0.0);
                public static final double ELEVATOR_FEEDER_POSITION_M = inchesToMeters(2);
                public static final double ELEVATOR_OVER_POSITION_M = inchesToMeters(20);
                public static final double ELEVATOR_NET_POSITION_M = inchesToMeters(64);
                public static final double ELEVATOR_L2_ALGAE_POSITION_M = inchesToMeters(19);
                public static final double ELEVATOR_L3_ALGAE_POSITION_M = inchesToMeters(38);
                public static final double ELEVATOR_PROCESSOR_POSITION_M = inchesToMeters(5);

                public enum ElevatorPosition {
                        kDOWN(ELEVATOR_BOTTOM_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_BOTTOM_POSITION_M)),
                        kMID(ELEVATOR_MID_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_MID_POSITION_M)),
                        kUP(ELEVATOR_TOP_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_TOP_POSITION_M)),
                        kL1(ELEVATOR_L1_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L1_POSITION_M)),
                        kL2(ELEVATOR_L2_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L2_POSITION_M)),
                        kAUTOL2(ELEVATOR_AUTO_L2_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L2_POSITION_M)),
                        kAUTOL3(ELEVATOR_AUTO_L3_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L3_POSITION_M)),
                        kAUTOL3SCORE(ELEVATOR_AUTO_SCORE_L3_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_SCORE_L3_POSITION_M)),
                        kAUTOL4(ELEVATOR_AUTO_L4_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_AUTO_L4_POSITION_M)),
                        kL3(ELEVATOR_L3_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L3_POSITION_M)),
                        kL4(ELEVATOR_L4_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_L4_POSITION_M)),
                        kALGAE(ELEVATOR_ALGAE_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_ALGAE_POSITION_M)),
                        kFEEDER(ELEVATOR_FEEDER_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_FEEDER_POSITION_M)),
                        kGROUNDCORAL(ELEVATOR_GROUND_CORAL_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_GROUND_CORAL_POSITION_M)),
                        kGROUNDALGAE(ELEVATOR_GROUND_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_GROUND_ALGAE_POSITION_M)),
                        kNET(ELEVATOR_NET_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_NET_POSITION_M)),
                        kPROCESSOR(ELEVATOR_PROCESSOR_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_PROCESSOR_POSITION_M)),
                        kL2ALGAE(ELEVATOR_L2_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_L2_ALGAE_POSITION_M)),
                        kL3ALGAE(ELEVATOR_L3_ALGAE_POSITION_M,
                                        Ratios.elevatorMetersToRotations(ELEVATOR_L3_ALGAE_POSITION_M)),
                        kOVER(ELEVATOR_OVER_POSITION_M, Ratios.elevatorMetersToRotations(ELEVATOR_OVER_POSITION_M));

                        public final double meters;
                        public final double rotations;

                        private ElevatorPosition(double meters, double rotations) {
                                this.meters = meters;
                                this.rotations = rotations;
                        }
                }

                public static final double PIVOT_L1_POSITION_D = 90.0;
                public static final double PIVOT_L23_POSITION_D = 55.0;
                // public static final double PIVOT_AUTO_L23_POSITION_D = 45.0;
                public static final double PIVOT_AUTO_L2_POSITION_D = 45.0;
                public static final double PIVOT_AUTO_L3_POSITION_D = 45.0;
                // public static final double PIVOT_AUTO_L3_POSITION_D = 30.0;
                public static final double PIVOT_AUTO_L4_POSITION_D = 70.0;
                public static final double PIVOT_AUTO_L4_SCORE_POSITION_D = 105.0;
                public static final double PIVOT_AUTO_L3_SCORE_POSITION_D = 90.0;
                public static final double PIVOT_AUTO_L2_SCORE_POSITION_D = 90.0;
                public static final double PIVOT_L4_POSITION_D = 70.0;
                public static final double PIVOT_UPRIGHT_POSITION_D = 45.0;
                public static final double PIVOT_GROUND_ALGAE_POSITION_D = 69.0;
                public static final double PIVOT_GROUND_CORAL_POSITION_FRONT_D = 135.0;
                public static final double PIVOT_GROUND_CORAL_POSITION_BACK_D = -135.0;
                public static final double PIVOT_GROUND_CORAL_PREP_BACK_D = -90;
                // public static final double PIVOT_DEFAULT_POSITION_D = 30.0;
                public static final double PIVOT_DEFAULT_POSITION_D = 0.0;
                public static final double PIVOT_PREP_POSITION_D = 30.0;
                public static final double PIVOT_FEEDER_POSITION_D = 39.37;
                public static final double PIVOT_NET_POSITION_D = 10.0;
                public static final double PIVOT_PROCESSOR_POSITION_D = 56.0;
                public static final double PIVOT_REEF_ALGAE_POSITION_D = 105.0;
                public static final double PIVOT_CLIMB_POSITION_D = 45.0;

                public enum PivotPosition {
                        kL1(PIVOT_L1_POSITION_D, Constants.degreesToRotations(PIVOT_L1_POSITION_D)),
                        kL23(PIVOT_L23_POSITION_D, Constants.degreesToRotations(PIVOT_L23_POSITION_D)),
                        kAUTOL2(PIVOT_AUTO_L2_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L2_POSITION_D)),
                        kAUTOL3(PIVOT_AUTO_L3_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L3_POSITION_D)),
                        kAUTOL4(PIVOT_AUTO_L4_POSITION_D, Constants.degreesToRotations(PIVOT_AUTO_L4_POSITION_D)),
                        kL4(PIVOT_L4_POSITION_D, Constants.degreesToRotations(PIVOT_L4_POSITION_D)),
                        kUP(PIVOT_UPRIGHT_POSITION_D, Constants.degreesToRotations(PIVOT_UPRIGHT_POSITION_D)),
                        kGROUNDALGAE(PIVOT_GROUND_ALGAE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_ALGAE_POSITION_D)),
                        kGROUNDCORALFRONT(PIVOT_GROUND_CORAL_POSITION_FRONT_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_POSITION_FRONT_D)),
                        kGROUNDCORALBACK(PIVOT_GROUND_CORAL_POSITION_BACK_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_POSITION_BACK_D)),
                        kGROUNDCORALPREPBACK(PIVOT_GROUND_CORAL_PREP_BACK_D,
                                        Constants.degreesToRotations(PIVOT_GROUND_CORAL_PREP_BACK_D)),
                        kNET(PIVOT_NET_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_NET_POSITION_D)),
                        kPROCESSOR(PIVOT_PROCESSOR_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_PROCESSOR_POSITION_D)),
                        kAUTOL2SCORE(PIVOT_AUTO_L2_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L2_SCORE_POSITION_D)),
                        kAUTOL3SCORE(PIVOT_AUTO_L3_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L3_SCORE_POSITION_D)),
                        kAUTOL4SCORE(PIVOT_AUTO_L4_SCORE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_AUTO_L4_SCORE_POSITION_D)),
                        kDEFAULT(PIVOT_DEFAULT_POSITION_D, Constants.degreesToRotations(PIVOT_DEFAULT_POSITION_D)),
                        kFEEDER(PIVOT_FEEDER_POSITION_D, Constants.degreesToRotations(PIVOT_FEEDER_POSITION_D)),
                        kREEFALGAE(PIVOT_REEF_ALGAE_POSITION_D,
                                        Constants.degreesToRotations(PIVOT_REEF_ALGAE_POSITION_D)),
                        kPREP(PIVOT_PREP_POSITION_D, Constants.degreesToRotations(PIVOT_PREP_POSITION_D)),
                        kCLIMB(PIVOT_CLIMB_POSITION_D, Constants.degreesToRotations(PIVOT_CLIMB_POSITION_D));

                        public final double degrees;
                        public final double rotations;

                        private PivotPosition(double degrees, double rotations) {
                                this.degrees = degrees;
                                this.rotations = rotations;
                        }
                }
        }

        // Vision constants (e.g. camera offsets)
        public static final class Vision {
                // Poses of all 16 AprilTags, {x, y, z, yaw, pitch}, in meters and radians
                public static final double[][] TAG_POSES = {
                                { inchesToMeters(657.37), inchesToMeters(25.8), inchesToMeters(58.5),
                                                Math.toRadians(126),
                                                Math.toRadians(0) },
                                { inchesToMeters(657.37), inchesToMeters(291.2), inchesToMeters(58.5),
                                                Math.toRadians(234),
                                                Math.toRadians(0) },
                                { inchesToMeters(455.15), inchesToMeters(317.15), inchesToMeters(51.25),
                                                Math.toRadians(270),
                                                Math.toRadians(0) },
                                { inchesToMeters(365.2), inchesToMeters(241.64), inchesToMeters(73.54),
                                                Math.toRadians(0),
                                                Math.toRadians(30) },
                                { inchesToMeters(365.2), inchesToMeters(75.39), inchesToMeters(73.54),
                                                Math.toRadians(0),
                                                Math.toRadians(30) },
                                { inchesToMeters(530.49), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(300),
                                                Math.toRadians(0) },
                                { inchesToMeters(546.87), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(0),
                                                Math.toRadians(0) },
                                { inchesToMeters(530.49), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(60),
                                                Math.toRadians(0) },
                                { inchesToMeters(497.77), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(120),
                                                Math.toRadians(0) },
                                { inchesToMeters(481.39), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(180),
                                                Math.toRadians(0) },
                                { inchesToMeters(497.77), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(240),
                                                Math.toRadians(0) },
                                { inchesToMeters(33.51), inchesToMeters(25.8), inchesToMeters(58.5), Math.toRadians(54),
                                                Math.toRadians(0) },
                                { inchesToMeters(33.51), inchesToMeters(291.2), inchesToMeters(58.5),
                                                Math.toRadians(306),
                                                Math.toRadians(0) },
                                { inchesToMeters(325.68), inchesToMeters(241.64), inchesToMeters(73.54),
                                                Math.toRadians(180),
                                                Math.toRadians(30) },
                                { inchesToMeters(325.68), inchesToMeters(75.39), inchesToMeters(73.54),
                                                Math.toRadians(180),
                                                Math.toRadians(30) },
                                { inchesToMeters(235.73), inchesToMeters(-0.15), inchesToMeters(51.25),
                                                Math.toRadians(90),
                                                Math.toRadians(0) },
                                { inchesToMeters(160.39), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(240),
                                                Math.toRadians(0) },
                                { inchesToMeters(144), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(180),
                                                Math.toRadians(0) },
                                { inchesToMeters(160.39), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(120),
                                                Math.toRadians(0) },
                                { inchesToMeters(193.1), inchesToMeters(186.83), inchesToMeters(12.13),
                                                Math.toRadians(60),
                                                Math.toRadians(0) },
                                { inchesToMeters(209.49), inchesToMeters(158.5), inchesToMeters(12.13),
                                                Math.toRadians(0),
                                                Math.toRadians(0) },
                                { inchesToMeters(193.1), inchesToMeters(130.17), inchesToMeters(12.13),
                                                Math.toRadians(300),
                                                Math.toRadians(0) }
                };

                public static final double[][] redSideReefTags = {
                                TAG_POSES[6 - 1], TAG_POSES[7 - 1], TAG_POSES[8 - 1], TAG_POSES[9 - 1],
                                TAG_POSES[10 - 1],
                                TAG_POSES[11 - 1] };

                public static final double[][] blueSideReefTags = {
                                TAG_POSES[6 + 11 - 1], TAG_POSES[7 + 11 - 1], TAG_POSES[8 + 11 - 1],
                                TAG_POSES[9 + 11 - 1],
                                TAG_POSES[10 + 11 - 1], TAG_POSES[11
                                                + 11 - 1] };

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
                        // Logger.recordOutput("std devs",
                        // -0.000277778 * Math.pow(dist, 3) + 0.00988095 * Math.pow(dist, 2) +
                        // 0.00444444 * dist + 0.0371429);
                        // return Math.max(1, a * Math.pow(dist, 2) + b);
                        return -0.00045928 * Math.pow(dist, 4) + 0.0069476 * Math.pow(dist, 3)
                                        - 0.0216241 * Math.pow(dist, 2)
                                        + 0.063534 * dist + 0.0317614;
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
        }

        // Gear ratios and conversions
        public static final class Ratios {

                // twist
                public static final double TWIST_GEAR_RATIO = 31.5;

                // pivot
                public static final double PIVOT_GEAR_RATIO = 91.4285714;

                // drive
                public static final double DRIVE_GEAR_RATIO = 6.12;
                public static final double STEER_GEAR_RATIO = 21.43;

                // elevator
                public static final double ELEVATOR_FIRST_STAGE = Constants.inchesToMeters(23.25);
                public static final double ELEVATOR_MOTOR_ROTATIONS_FOR_FIRST_STAGE = 20.425781;
                public static final double ELEVATOR_MOTOR_ROTATIONS_PER_METER = ELEVATOR_MOTOR_ROTATIONS_FOR_FIRST_STAGE
                                * (1 / ELEVATOR_FIRST_STAGE);

                public static double elevatorRotationsToMeters(double rotations) {
                        return rotations / ELEVATOR_MOTOR_ROTATIONS_PER_METER;
                }

                public static double elevatorMetersToRotations(double meters) {
                        return meters * ELEVATOR_MOTOR_ROTATIONS_PER_METER;
                }
        }

        // Can info such as IDs
        public static final class CANInfo {
                public static final String CANBUS_NAME = "Canivore";

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

                // Lights
                public static final int CANDLE_ID_0 = 0;
                public static final int CANDLE_ID_1 = 1;
                public static final int CANDLE_ID_2 = 2;

                // Elevator
                public static final int MASTER_ELEVATOR_MOTOR_ID = 9;
                public static final int FOLLOWER_ELEVATOR_MOTOR_ID = 10;

                // Pivot
                public static final int PIVOT_MOTOR_ID = 11;
                public static final int PIVOT_CANCODER_ID = 5;

                // Twist
                public static final int TWIST_MOTOR_ID = 12;
                public static final int TWIST_CANCODER_ID = 6;

                // Intake
                public static final int INTAKE_MOTOR_ID = 13;

                // Climber
                public static final int CLIMBER_ROLLER_MOTOR_ID = 14;
                public static final int CLIMBER_PIVOT_MOTOR_ID = 15;
        }

        // Misc. controller values
        public static final class OperatorConstants {
                public static final double RIGHT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_TRIGGER_DEADZONE = 0.1;
                public static final double LEFT_STICK_DEADZONE = 0.03;
                public static final double RIGHT_STICK_DEADZONE = 0.05;
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
                // System.out.println("x1: " + x1 + ", y1: " + y1 + ", x2: " + x2 + ", y2: " +
                // y2);
                double deltaX = x2 - x1;
                double deltaY = y2 - y1;

                double angleInRadians = Math.atan2(deltaY, deltaX);

                double angleInDegrees = Math.toDegrees(angleInRadians);

                double standardizeAngleDegrees = standardizeAngleDegrees(angleInDegrees);

                // if (y1 > y2) {
                // System.out.println("running");
                return 180 + standardizeAngleDegrees;
                // }
                // System.out.println("2");
                // double temp = 180 - standardizeAngleDegrees;
                // double j = 180 - temp;
                // return 180 + j;
                // }
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
                // System.out.println("initial angle degrees" + angleDegrees);
                if (angleDegrees >= 0 && angleDegrees < 360) {
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else if (angleDegrees < 0) {
                        while (angleDegrees < 0) {
                                angleDegrees += 360;
                        }
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else if (angleDegrees >= 360) {
                        while (angleDegrees >= 360) {
                                angleDegrees -= 360;
                        }
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                } else {
                        // System.out.println("Weird ErroR");
                        // System.out.println("standardized angle degrees" + angleDegrees);
                        return angleDegrees;
                }
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
}
