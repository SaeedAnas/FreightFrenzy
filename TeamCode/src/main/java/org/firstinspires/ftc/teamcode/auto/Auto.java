package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.ColorFilterPipeline;

@Config
@Autonomous(group = "blue")
public class Auto extends LinearOpMode {
    Robot robot;
    SampleMecanumDriveCancelable drive;

    enum State {
        TO_HUB_START,
        INTAKE,
        TO_HUB,
        PARK,
        IDLE
    }

    public enum Level {
        TOP,
        MIDDLE,
        BOTTOM
    }

    State currentState = State.IDLE;
    Level level = Level.TOP;

    public static final Pose2d blueStartingPosition = new Pose2d(9, 62.8, toRadians(90));
    private static int currentCycle = 0;
    public static int maxCycles = 3;

    public static boolean reversed = false;
    public static double x = 20;
    public static double y = 63.5;
    public static double h = 0;

    public static double topPosX = -11;
    public static double topPosY = 40.5;

    public static double middlePosX = -13;
    public static double middlePosY = 50;

    public static double bottomPosX = -13;
    public static double bottomPosY = 55;

    public static double lx = 10;
    public static double ly = 73.5;

    public void intake() {
        TrajectorySequence in = drive.trajectorySequenceBuilder(new Pose2d(-13, 44, 270))
//                .setReversed(true)
//                .setVelConstraint((v, a, b, c) -> 20)
                .setReversed(reversed)
                .splineToSplineHeading(new Pose2d(x, y, h), h)
                .lineTo(new Vector2d(38, 63.5))
                .addTemporalMarker(() -> {
                    back();
                })

                .build();

        drive.followTrajectorySequenceAsync(in);
    }

    public void back() {
        TrajectorySequence to = drive.trajectorySequenceBuilder(new Pose2d(38, 63.5, 0))
                .lineTo(new Vector2d(25, 63.5))
                .lineTo(new Vector2d(20, 63.5))
                .splineTo(new Vector2d(-11, 44), toRadians(270))
                .addTemporalMarker(() -> {
                    intake();
                })
                .build();

        drive.followTrajectorySequenceAsync(to);
    }


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.drive;

        drive.setPoseEstimate(blueStartingPosition);
        PoseStorage.currentPose = blueStartingPosition;

        robot.stream();


        while (!isStarted() && !isStopRequested()) {
            double pos = ColorFilterPipeline.xPos;
            double width = ColorFilterPipeline.boxWidth;
            telemetry.addData("xPos", pos);
            telemetry.addData("width", width);

            if (pos > 400) {
                telemetry.addData("Level", "TOP");
                level = Level.TOP;
            } else if (pos < 400 && pos > 100) {
                telemetry.addData("Level", "MIDDLE");
                level = Level.MIDDLE;
            } else {
                telemetry.addData("Level", "BOTTOM");
                level = Level.BOTTOM;
            }
            telemetry.update();
        }

        double yPos;
        double xPos;
        switch (level) {
            case MIDDLE: {
                xPos = middlePosX;
                yPos = middlePosY;
                break;
            }
            case BOTTOM: {
                xPos = bottomPosX;
                yPos = bottomPosY;
                break;
            }
            default: {
                xPos = topPosX;
                yPos = topPosY;
                break;
            }
        }


        TrajectorySequence t = drive.trajectorySequenceBuilder(blueStartingPosition)
                .addTemporalMarker(0.3, () -> {
                    switch (level) {
                        case MIDDLE: {
                            robot.arm.middle();
                            break;
                        }
                        case BOTTOM: {
                            robot.arm.bottom();
                            break;
                        }
                        default: {
                            robot.arm.topAuto();
                        }
                    }
//                    robot.arm.topAuto();

                    System.out.println("armUp");
                })
                .lineToConstantHeading(new Vector2d(xPos, yPos))
                .addTemporalMarker(() -> {
                    robot.arm.dump();
                    robot.scheduleTask(() -> robot.arm.closeArm(), 1000);
                    System.out.println("outtake");
                })
                .setVelConstraint((v, a, b, c) -> 20)
//                .lineTo(new Vector2d(xPos, yPos + 5))
                .waitSeconds(1)
                .lineTo(new Vector2d(xPos, yPos + 5))
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(lx, ly, toRadians(0)))
//                .lineTo(new Vector2d(-13, 40))
//                .setReversed(false)
//                .splineTo(new Vector2d(30, 65.5), toRadians(0))
//                .addTemporalMarker(() -> {
//                    robot.intake();
//                })
                .lineTo(new Vector2d(35, ly))
//                .lineTo(new Vector2d(44, 64.5))
////
//                .lineTo(new Vector2d(20, 64.5))
//                .addTemporalMarker(() -> {
//                    robot.arm.topAuto();
//                })
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//                .addTemporalMarker(() -> {
//                    robot.arm.dump();
//                    robot.scheduleTask(() -> robot.arm.closeArm(), 1000);
//                    System.out.println("outtake");
//                })
//
////
//                .setReversed(false)
//                .splineTo(new Vector2d(20, 64.5), toRadians(0))
//                .addTemporalMarker(() -> {
//                    robot.intake();
//                })
//                .lineTo(new Vector2d(44, 64.5))
//
//                .lineTo(new Vector2d(20, 64.5))
//                .addTemporalMarker(() -> {
//                    robot.arm.topAuto();
//                })
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//                .addTemporalMarker(() -> {
//                    robot.arm.dump();
//                    robot.scheduleTask(() -> robot.arm.closeArm(), 1000);
//                    System.out.println("outtake");
//                })
//
////
//                .setReversed(false)
//                .splineTo(new Vector2d(20, 64.5), toRadians(0))
////                .addTemporalMarker(() -> {
////                    robot.intake();
////                })
//                .lineTo(new Vector2d(44, 64.5))

//
//                .lineTo(new Vector2d(20, 63.5))
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//
//                .setReversed(false)
//                .splineTo(new Vector2d(20, 63.5), toRadians(0))
//                .lineTo(new Vector2d(44, 63.5))
//
//                .lineTo(new Vector2d(20, 63.5))
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//
//                .setReversed(false)
//                .splineTo(new Vector2d(20, 63.5), toRadians(0))
//                .lineTo(new Vector2d(40, 63.5))

//                .lineTo(new Vector2d(20, 63.5))
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//
//                .setReversed(false)
//                .splineTo(new Vector2d(20, 63.5), toRadians(0))
//                .lineTo(new Vector2d(44, 63.5))

                .build();

        waitForStart();

        robot.stopStreaming();

//            telemetry.addData("xPos", ColorFilterPipeline.xPos);
        telemetry.update();

        if (isStopRequested()) return;

//        toHubStart(level);

        drive.followTrajectorySequenceAsync(t);

        while (opModeIsActive() && !isStopRequested()) {
//            switch (currentState) {
//                case TO_HUB_START: {
//                    if (!drive.isBusy())
//                        toIntake();
//                    break;
//                }
//                case INTAKE: {
//                    if (!drive.isBusy()) // TODO Or if intake has block
//                        toHub();
////                        currentState = State.IDLE;
//                    break;
//                }
//                case TO_HUB: {
//                    if (!drive.isBusy()) {
////                        currentCycle++;
////                        if (currentCycle < maxCycles) {
////                            toIntake();
////                        } else {
////                            toPark();
////                        }
//                        toIntake();
//                    }
//                    break;
//                }
//                case PARK: {
//                    if (!drive.isBusy()) {
//                        currentState = State.IDLE;
//                    }
//                }
//                case IDLE: {
//                    break;
//                }
//            }

//            telemetry.addData("AutoState", currentState);

            robot.updateAuto();
        }

    }

    private void toPark() {
        Trajectory park = drive.trajectoryBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                .build();

        TrajectorySequence toPark = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTemporalMarker(() -> {
                    System.out.println("arm down");
                })
                .addTrajectory(park)
                .build();

        currentState = State.PARK;
        drive.followTrajectorySequenceAsync(toPark);
    }

    private void toHub() {
        Trajectory hub = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineTo(new Vector2d(40, 64.75))
//                .lineToSplineHeading(new Pose2d(40, 63.1, toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    Pose2d p = new Pose2d(10, 61.1, toRadians(0));
//                    drive.setPoseEstimate(p);
//                    PoseStorage.currentPose = p;
//                })
//
//                .addTemporalMarker(0.5, () -> {
//                    System.out.println("armUp");
//                })
                .splineToConstantHeading(new Vector2d(5, 62), toRadians(0))
                .splineTo(new Vector2d(-11, 44), toRadians(270))
//                                        .splineToSplineHeading(new Pose2d(5, 64, toRadians(0)), toRadians(0))
//                .splineToConstantHeading(new Vector2d(13, 64), 0)
//                .splineToConstantHeading(new Vector2d(44, 64), 0)

//                .lineTo(new Vector2d(15, 64.1))
//                .lineTo(new Vector2d(5, 62))
//                .splineTo(new Vector2d(-13, 44), toRadians(270))
//                .splineTo(new Vector2d(-10, 40), toRadians(265))
                .build();

        TrajectorySequence toHub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .setReversed(true)
//                .addTrajectory(hub)
//                .addTemporalMarker(() -> {
//                    System.out.println("outtake");
//                })
//                .setVelConstraint((v, pose2d1, pose2d2, pose2d3) -> 70)
//                .lineTo(new Vector2d(10, 61.1))
//                .addTemporalMarker(() -> {
//                    Pose2d p = new Pose2d(10, 61.1, toRadians(0));
//                    drive.setPoseEstimate(p);
//                    PoseStorage.currentPose = p;
//                })
//                .splineToConstantHeading(new Vector2d(5, 55), toRadians(0))
//
//                .splineTo(new Vector2d(-11, 44), toRadians(270))
//                .addTrajectory(hub)
                .lineTo(new Vector2d(10, 62))
                .lineTo(new Vector2d(0, 50))
                .build();

        currentState = State.TO_HUB;
        drive.followTrajectorySequenceAsync(toHub);
    }

    private void toIntake() {
        Trajectory intake = drive.trajectoryBuilder(drive.getPoseEstimate())
//                .lineToConstantHeading(new Vector2d(-15, 60))
//                .lineToSplineHeading(new Pose2d(10, 63.1, toRadians(0)))
//                .splineTo(new Vector2d(0, 61.1), toRadians(0))
//                .lineToSplineHeading(new Pose2d(10, 61.1, toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    Pose2d p = new Pose2d(10, 61.1, toRadians(0));
//                    drive.setPoseEstimate(p);
//                    PoseStorage.currentPose = p;
//                })
                .splineTo(new Vector2d(10, 62), toRadians(0))
//                                        .splineToSplineHeading(new Pose2d(5, 64, toRadians(0)), toRadians(0))
//                .splineToConstantHeading(new Vector2d(15, 62), 0)
//                .splineToConstantHeading(new Vector2d(30, 62), 0)
//                                        .splineTo(new Vector2d(25, 64.75), toRadians(0))

//                .splineTo(new Vector2d(40, 61.1), toRadians(0))
//                .forward(30)
//                .forward(40)
//                .splineToLinearHeading(new Pose2d(10, 64.1, toRadians(0)), toRadians(0))
//                .splineTo(new Vector2d(30, 64.1), toRadians(0))
//                .splineToSplineHeading(new Pose2d(15, 64.1, toRadians(0)), toRadians(0))
                .addDisplacementMarker(() -> {
                    System.out.println("intake");
//                    toHub();
                })
//                .lineTo(new Vector2d(45, 64.1))
                .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                .addTemporalMarker(() -> {
//                    System.out.println("arm down");
//                })
//                .setVelConstraint((v, pose2d1, pose2d2, pose2d3) -> 50)
//                .splineTo(new Vector2d(10, 63), toRadians(0))
//                .lineTo(new Vector2d(40, 63.2))
//                .addTrajectory(intake)
                .splineTo(new Vector2d(10, 62), toRadians(0))
                .lineTo(new Vector2d(30, 62))
//                .addTrajectory(intake)
//                .lineTo(new Vector2d(45, 61.1))
//                .addTemporalMarker(() -> {
//                    drive.followTrajectorySequenceAsync(
//                            drive.trajectorySequenceBuilder(intake.end())
//                                    .setVelConstraint((v, pose2d1, pose2d2, pose2d3) -> 40)
//                                    .lineTo(new Vector2d(45, 61.1))
//                                    .build()
//                    );
//                })
//                .lineTo(new Vector2d(45, 64.1))
                .build();
        TrajectorySequence i = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(13, 64), toRadians((0)))
                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                .build();

        currentState = State.INTAKE;
        drive.followTrajectorySequenceAsync(i);


//        drive.followTrajectorySequenceAsync(toIntake);
    }

    private void toHubStart(Level l) {
        Trajectory hubStart;
        switch (l) {
            case MIDDLE: {
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-13, 44))
                        .build();
                break;
            }
            case BOTTOM: {
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-13, 44))
                        .build();
                break;
            }
            default: {
                // TOP Trajectory
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-11, 44))
                        .build();
            }
        }
        TrajectorySequence toHubStart = drive.trajectorySequenceBuilder(blueStartingPosition)
                .addTemporalMarker(() -> {
                    System.out.println("armUp");
                })
//                .addTrajectory(hubStart)
                .lineToConstantHeading(new Vector2d(-11, 44))
                .addTemporalMarker(() -> {
                    System.out.println("outtake");
                })
//                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    toIntake();
                })
                .build();

        currentState = State.TO_HUB_START;
        drive.followTrajectorySequenceAsync(toHubStart);
    }


}
