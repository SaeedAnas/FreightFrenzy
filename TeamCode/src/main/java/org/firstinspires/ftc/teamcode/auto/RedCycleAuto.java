package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.TestPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(group = "cycle")
public class RedCycleAuto extends LinearOpMode {
    Robot robot;
    SampleMecanumDrive drive;

    OpenCvCamera camera;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272; // not sure wtf these are supposed to be
    double fy = 578.272;
    double cx = 1280 / 2.0;
    double cy = 720 / 2.0;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public static int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family

    public static double first = -0.5;
    public static double second = 0.5;

    AprilTagDetection tagOfInterest = null;


    public static double x = 14;
    public static double y = 63.75;

    enum State {
        TO_HUB_START,
        INTAKE,
        TO_HUB,
        IDLE
    }

    public static double top = 40;
    public static double middle = 49;
    public static double bottom = 48;

    public enum Level {
        TOP(top),
        MIDDLE(middle),
        BOTTOM(bottom);

        double y;

        Level(double y) {
            this.y = y;
        }
    }

    State currentState = State.IDLE;
    Level level = Level.TOP;

    public static double maxCycles = 2;
    public static double currentCycle = 0;

    private static final Pose2d blueStartingPosition = new Pose2d(8.34375, 65.375, toRadians(90));

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        TestPipeline aprilTagDetectionPipeline = new TestPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Initialize Robot
        robot = new Robot(hardwareMap);

        // Initialize Path Follower
        drive = new SampleMecanumDrive(hardwareMap);

        // Set initial pose
        drive.setPoseEstimate(blueStartingPosition);


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        waitForStart();
        camera.closeCameraDevice();

        if (isStopRequested()) return;

        // Set the current state to TO_HUB_START
        currentState = State.TO_HUB_START;
        toStart(level);


        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TO_HUB_START: {
                    if (!drive.isBusy()) {
                        currentState = State.INTAKE;
                        intake();
                    }
                    break;
                }
                case INTAKE: {
                    if (robot.intake.hasFreight() || !drive.isBusy()) {
                        robot.intake.setState(Intake.State.OFF);
                        currentState = State.TO_HUB;
                        currentCycle++;
                        toHub();
                    }
                    break;
                }
                case TO_HUB: {
                    if (!drive.isBusy()) {
                        currentState = State.INTAKE;
                        intake();
//                        drive.followTrajectorySequenceAsync(intake);
                    }
                    break;
                }
                case IDLE: {
                    break;
                }
            }

            drive.update();
            robot.updateAuto();


            PoseStorage.currentPose = drive.getPoseEstimate();

            telemetry.addData("State", currentState);
            telemetry.update();
        }

    }

    public void toStart(Level l) {
        if (l == Level.TOP) {
            TrajectorySequence toHubStart = drive.trajectorySequenceBuilder(blueStartingPosition)
                    .addTemporalMarker(() -> {
                        robot.arm.top();
                    })
                    .lineToConstantHeading(new Vector2d(-13, level.y))
                    .addTemporalMarker(() -> {
                        robot.arm.dump();
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        robot.arm.closeArm();
                    })
                    .addTemporalMarker(() -> {
                        currentState = State.INTAKE;
                        robot.autoIntake();
                    })
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
                    .splineTo(new Vector2d(40, 65.75), toRadians(0))
                    .resetVelConstraint()
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 25)
                    .lineTo(new Vector2d(58, 63.75))
                    .resetVelConstraint()
                    .build();

            drive.followTrajectorySequenceAsync(toHubStart);
        } else if (l == Level.MIDDLE) {
            TrajectorySequence toHubStart = drive.trajectorySequenceBuilder(blueStartingPosition)
                    .addTemporalMarker(() -> {
                        robot.arm.middle();
                    })
                    .lineToConstantHeading(new Vector2d(-13, level.y))
                    .addTemporalMarker(() -> {
                        robot.arm.dump();
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        robot.arm.closeArm();
                    })
                    .addTemporalMarker(() -> {
                        currentState = State.INTAKE;
                        robot.autoIntake();
                    })
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
                    .splineTo(new Vector2d(45, 65.75), toRadians(0))
                    .resetVelConstraint()
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 25)
                    .lineTo(new Vector2d(58, 63.75))
                    .resetVelConstraint()
                    .build();
            drive.followTrajectorySequenceAsync(toHubStart);
        } else {
            TrajectorySequence toHubStart = drive.trajectorySequenceBuilder(blueStartingPosition)
                    .addTemporalMarker(() -> {
                        robot.arm.bottom();
                    })
                    .lineToConstantHeading(new Vector2d(-13, level.y))
                    .addTemporalMarker(() -> {
                        robot.arm.dump();
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        robot.arm.closeArm();
                    })
                    .addTemporalMarker(() -> {
                        currentState = State.INTAKE;
                        robot.autoIntake();
                    })
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 45)
                    .splineTo(new Vector2d(45, 65.75), toRadians(0))
                    .resetVelConstraint()
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 25)
                    .lineTo(new Vector2d(58, 63.75))
                    .resetVelConstraint()
                    .build();

            drive.followTrajectorySequenceAsync(toHubStart);
        }

    }

    public void toHub() {
//        TrajectorySequence hub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                // Dump Held Block
//                .lineToConstantHeading(new Vector2d(-13, 45))
//
////                .addDisplacementMarker(() -> System.out.println("Lift out"))
//                .lineToSplineHeading(new Pose2d(14, 63.75, toRadians(0)))
//                .lineTo(new Vector2d(56, 63.5))
//                .waitSeconds(0.5)
//                .lineTo(new Vector2d(14, 63.5))
//                .lineToSplineHeading(new Pose2d(-5, 45, toRadians(70)))
//                .lineToSplineHeading(new Pose2d(14, 63.75, toRadians(0)))
//                .lineTo(new Vector2d(56, 63.5))
//                .waitSeconds(0.5)
//                .lineTo(new Vector2d(14, 63.5))
//                .lineToSplineHeading(new Pose2d(-5, 45, toRadians(70)))
//                .lineToSplineHeading(new Pose2d(14, 63.75, toRadians(0)))
//                .lineTo(new Vector2d(56, 63.5))
//                .waitSeconds(0.5)
//                .lineTo(new Vector2d(14, 63.5))
//                .lineToSplineHeading(new Pose2d(-5, 45, toRadians(70)))
//                .lineToSplineHeading(new Pose2d(14, 63.75, toRadians(0)))
//                .lineTo(new Vector2d(56, 63.5))
//                .waitSeconds(0.5)
//                .lineTo(new Vector2d(14, 63.5))
//                .lineToSplineHeading(new Pose2d(-5, 45, toRadians(70)))
//                .build();

        TrajectorySequence hub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
//                .lineTo(new Vector2d(20, 63.5))
//                .splineTo(new Vector2d(0, 38), toRadians(245))
                .lineTo(new Vector2d(30, 66))
                .addTemporalMarker(() -> {
                    robot.arm.top();
                })
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 40)
//                .splineTo(new Vector2d(0, 38), toRadians(245))
                .splineTo(new Vector2d(-8, 43), toRadians(253))
                .addTemporalMarker(() -> {
                    robot.arm.dump();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.closeArm();
                })
//                .splineTo(new Vector2d(44, 63.75), toRadians(0))
                .addTemporalMarker(() -> {
                    currentState = State.INTAKE;
                    robot.autoIntake();
                })
                .splineTo(new Vector2d(15, 69), toRadians(0))
                .resetVelConstraint()
                .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 35)
                .lineTo(new Vector2d(58, 67.5))
//                .splineTo(new Vector2d(58, 67.5), toRadians(-5))
                .resetVelConstraint()
                .build();

        drive.followTrajectorySequenceAsync(hub);
    }

    public void intake() {
        robot.intake();
        TrajectorySequence intake = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .lineTo(new Vector2d(58, 64.5))
//                .splineTo(new Vector2d(58, 61), toRadians(0))
                .build();

        drive.followTrajectorySequenceAsync(intake);

    }

    void tagToTelemetry(AprilTagDetection detection) {
        if (tagOfInterest.pose.x <= first) {
            level = Level.TOP;
        } else if (tagOfInterest.pose.x >= first && tagOfInterest.pose.x <= second) {
            level = Level.MIDDLE;
        }
        telemetry.addData("Level", level);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}

