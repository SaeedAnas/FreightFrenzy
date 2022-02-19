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

    private static final Pose2d blueStartingPosition = new Pose2d(8.34375, 65.375, toRadians(90));
    private static int currentCycle = 0;
    public static int maxCycles = 3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.drive;

        drive.setPoseEstimate(blueStartingPosition);

        waitForStart();

        if (isStopRequested()) return;

        toHubStart(level);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case TO_HUB_START: {
                    if (!drive.isBusy())
                        toIntake();
                    break;
                }
                case INTAKE: {
                    if (!drive.isBusy()) // TODO Or if intake has block
                        toHub();
                    break;
                }
                case TO_HUB: {
                    if (!drive.isBusy()) {
                        currentCycle++;
                        if (currentCycle < maxCycles) {
                            toIntake();
                        } else {
                            toPark();
                        }
                    }
                    break;
                }
                case PARK: {
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                }
                case IDLE: {
                    break;
                }
            }

            telemetry.addData("State", currentState);

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
        Trajectory hub = drive.trajectoryBuilder(PoseStorage.currentPose)
                .lineTo(new Vector2d(40, 64.75))
                .addTemporalMarker(0.5, () -> {
                    System.out.println("armUp");
                })
                .splineTo(new Vector2d(-13, 40), toRadians(270))
//                .splineTo(new Vector2d(-10, 40), toRadians(265))
                .build();

        TrajectorySequence toHub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .setReversed(true)
                .addTrajectory(hub)
                .addTemporalMarker(() -> {
                    System.out.println("outtake");
                })
                .waitSeconds(0.5)
                .build();

        currentState = State.TO_HUB;
        drive.followTrajectorySequenceAsync(toHub);
    }

    private void toIntake() {
        Trajectory intake = drive.trajectoryBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(40, 64.75), toRadians(0))
                .addDisplacementMarker(() -> {
                    System.out.println("intake");
                })
                .lineTo(new Vector2d(60, 64.75))
                .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTemporalMarker(() -> {
                    System.out.println("arm down");
                })
                .setReversed(false)
                .addTrajectory(intake)
                .build();

        currentState = State.INTAKE;
        drive.followTrajectorySequenceAsync(toIntake);
    }

    private void toHubStart(Level l) {
        Trajectory hubStart;
        switch (l) {
            case MIDDLE: {
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-13, 49))
                        .build();
                break;
            }
            case BOTTOM: {
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-13, 48))
                        .build();
                break;
            }
            default: {
                // TOP Trajectory
                hubStart = drive.trajectoryBuilder(blueStartingPosition)
                        .lineToConstantHeading(new Vector2d(-13, 40))
                        .build();
            }
        }
        TrajectorySequence toHubStart = drive.trajectorySequenceBuilder(blueStartingPosition)
                .addTemporalMarker(() -> {
                    System.out.println("armUp");
                })
                .addTrajectory(hubStart)
                .addTemporalMarker(() -> {
                    System.out.println("outtake");
                })
                .waitSeconds(0.5)
                .build();

        currentState = State.TO_HUB_START;
        drive.followTrajectorySequenceAsync(toHubStart);
    }


}
