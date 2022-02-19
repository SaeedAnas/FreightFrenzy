package com.example.meepmeeptesting;

import static com.noahbres.meepmeep.core.ExtensionsKt.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {


    public static double MAX_VEL = 75;
    public static double MAX_ACCEL = 35;
    public static double MAX_ANG_VEL = toRadians(200);
    public static double MAX_ANG_ACCEL = toRadians(200);
    public static double TRACK_WIDTH = 12.1;


    //length = 17.3125
    //width = 13.25


    private static final Pose2d blueStartingPosition = new Pose2d(8.34375, 65.375, toRadians(90));
    private static final Pose2d blueDuckPosition = new Pose2d(-33.96875, 65.375, toRadians(90));
    private static final Pose2d redStartingPosition =
            blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(), toRadians(-90));

    private static final Pose2d blueStartingPositionDuck = new Pose2d(-31.96875, 65.375, toRadians(0));
    private static final Pose2d redStartingPositionDuck =
            blueStartingPosition.copy(blueStartingPosition.getX(), -blueStartingPosition.getY(),
                    Angle.normDelta(blueStartingPosition.getHeading() + toRadians(180)));

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");
        MeepMeep mm = new MeepMeep(600);
        double width = 13;
        double height = 13.625;
        RoadRunnerBotEntity newCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStartingPosition)
                                .addTemporalMarker(() -> {
                                    System.out.println("armUp");
                                })
                                .lineToConstantHeading(new Vector2d(-13, 40))
                                .addTemporalMarker(() -> {
                                    System.out.println("armDown");
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    System.out.println("closeArm");
                                })
                                // Intake
                                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                                .lineTo(new Vector2d(60, 64.75))
                                // Outtake
                                .setReversed(true)
                                .lineTo(new Vector2d(40, 64.75))
                                .splineTo(new Vector2d(-13, 40), toRadians(270))
                                // Intake
                                .setReversed(false)
                                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                                .lineTo(new Vector2d(60, 64.75))
                                // Outake
                                .lineTo(new Vector2d(40, 64.75))
                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 40), toRadians(265))
                                // Intake
                                .setReversed(false)
                                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                                .lineTo(new Vector2d(60, 64.75))
                                // Outake
                                .lineTo(new Vector2d(40, 64.75))
                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 40), toRadians(265))
                                // Intake
                                .setReversed(false)
                                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                                .lineTo(new Vector2d(60, 64.75))
                                // Outake
                                .lineTo(new Vector2d(40, 64.75))
                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 40), toRadians(265))
                                .build()
                );

        Pose2d intakePos = new Pose2d(40, 38, toRadians(30));
        RoadRunnerBotEntity hubRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(intakePos)
//                                .setReversed(true)
                                        .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                                        .lineTo(new Vector2d(30, 64.75))
                                        .splineTo(new Vector2d(-13, 40), toRadians(270))
                                        .build()
                );

        RoadRunnerBotEntity hubRight = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(intakePos)
                                .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                                .lineTo(new Vector2d(20, 64.75))
                                .splineTo(new Vector2d(8, 42.8), toRadians(270))
                                .lineToSplineHeading(new Pose2d(8, 22.8, toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity hubLeft = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(intakePos)
                                .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                                .lineTo(new Vector2d(20, 64.75))
                                .splineTo(new Vector2d(-31.5, 42.8), toRadians(270))
                                .lineToSplineHeading(new Pose2d(-31.5, 22.8, toRadians(180)))
                                .build()
                );

        Pose2d hubPos = new Pose2d(-13, 40, toRadians(270));
        RoadRunnerBotEntity intakeRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(hubPos)
                                .setReversed(true)
                                .splineTo(new Vector2d(25, 64.75), toRadians(0))
                                .lineTo(new Vector2d(30, 64.75))
                                .build()
                );

        Pose2d hubPosRight = new Pose2d(8, 22.8, toRadians(0));
        RoadRunnerBotEntity intakeRouteRight = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(hubPosRight)
//                                .lineTo(new Vector2d(8, 30))
                                        .splineTo(new Vector2d(25, 64.75), toRadians(0))
                                        .lineTo(new Vector2d(35, 64.75))
                                        .build()
                );

        Pose2d hubPosLeft = new Pose2d(-31.5, 22.8, toRadians(180));
        RoadRunnerBotEntity intakeRouteLeft = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(hubPosLeft)
                                .splineTo(new Vector2d(-8, 60), toRadians(0))
                                .splineTo(new Vector2d(25, 64.75), toRadians(0))
                                .lineTo(new Vector2d(35, 64.75))
                                .build()
                );


        RoadRunnerBotEntity blueDuckRoute2 = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(blueDuckPosition)
                                        .lineToSplineHeading(new Pose2d(-30, 37, toRadians(155)))
                                        .addTemporalMarker(() -> {
                                            System.out.println("armDown");
                                        })
                                        .lineToConstantHeading(new Vector2d(-60, 36))
//                            .splineTo(new Vector2d(60, 30), toRadians(0))
                                        .build()
                );

        RoadRunnerBotEntity redCycleRouteSimple = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPosition)
                                .addTemporalMarker(() -> {
                                    System.out.println("armUp");
                                })
                                .lineToConstantHeading(new Vector2d(-13, -55))
                                .addTemporalMarker(() -> {
                                    System.out.println("armDown");
                                })
                                .waitSeconds(0.5)
                                .addTemporalMarker(() -> {
                                    System.out.println("closeArm");
                                })
                                .splineTo(new Vector2d(40, -65.75), toRadians(0))
                                .build()
                );


        RoadRunnerBotEntity redCycleRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPosition)
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(0))
                                .splineToConstantHeading(new Vector2d(50, -64), toRadians(0))
                                .addDisplacementMarker(() -> System.out.println("Intaking"))
                                .back(7, getVelocityConstraint(5, toRadians(200), 12.1))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Freight Found, Relocalizing"))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(13, -64.5), toRadians(180))
                                .splineToConstantHeading(new Vector2d(-10, -58), toRadians(160))
                                .setReversed(true)
                                .build()
                );

        RoadRunnerBotEntity blueDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueStartingPositionDuck)
                                .lineToConstantHeading(new Vector2d(-10, 58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .lineToConstantHeading(new Vector2d(-35, 50))
                                .lineToConstantHeading(new Vector2d(-50, 40))
                                .turn(toRadians(-30))
                                .lineToConstantHeading(new Vector2d(-57, 55))
                                .waitSeconds(3)
                                .forward(10)
                                .turn(toRadians(30))
                                .forward(20)
                                .strafeLeft(10)
                                .back(20,
                                        getVelocityConstraint(10, toRadians(180), 13)
                                )
                                .lineToConstantHeading(new Vector2d(-10, 58))
                                .waitSeconds(3)
                                .back(20)
                                .lineToLinearHeading(new Pose2d(-60, 40, toRadians(0)))
                                .build()
                );

        RoadRunnerBotEntity redDuckRoute = new DefaultBotBuilder(mm)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(width, height)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redStartingPositionDuck)
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift out"))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(() -> System.out.println("Lift in"))
                                .lineToConstantHeading(new Vector2d(-35, -50))
                                .lineToConstantHeading(new Vector2d(-50, -40))
                                .turn(toRadians(-150))
                                .lineToConstantHeading(new Vector2d(-57, -55))
                                .waitSeconds(3)
                                .forward(10)
                                .turn(toRadians(-30))
                                .forward(20)
                                .strafeRight(10)
                                .back(20,
                                        getVelocityConstraint(10, toRadians(180), 13)
                                )
                                .lineToConstantHeading(new Vector2d(-10, -58))
                                .waitSeconds(3)
                                .back(20)
                                .lineToLinearHeading(new Pose2d(-60, -40, toRadians(0)))
                                .build()
                );


        mm
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
//                .addEntity(newCycleRoute)
                .addEntity(hubRoute)
                .addEntity(intakeRoute)
                .addEntity(hubRight)
                .addEntity(hubLeft)
                .addEntity(intakeRouteRight)
                .addEntity(intakeRouteLeft)
                .start();

    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}