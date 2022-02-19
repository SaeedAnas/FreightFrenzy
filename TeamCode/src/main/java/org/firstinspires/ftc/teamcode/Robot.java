package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.ArmDumpy;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.Dumpy;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SecondArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.vision.Webcam;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot {
    public Arm arm;
    public SecondArm secondArm;
    public ArmDumpy armDumpy;
    public Intake intake;
    public Dumpy dumpy;
    public DistanceSensor sensor;
    public SampleMecanumDriveCancelable drive;
    public Webcam webcam;
    Telemetry telemetry;

    ExecutorService executor = Executors.newFixedThreadPool(10);

    public enum State {
        DRIVER_CONTROL,
        AUTO_CONTROL;
    }

    private State currentState;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        arm = new Arm(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        sensor = new DistanceSensor(hardwareMap);
        webcam = new Webcam(hardwareMap);
        dumpy = new Dumpy(hardwareMap, this);
        secondArm = new SecondArm(hardwareMap, this);
        armDumpy = new ArmDumpy(hardwareMap, this);

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        this.telemetry = telemetry;

        currentState = State.DRIVER_CONTROL;
    }

    public void shutdown() {
        executor.shutdown();
    }

    public void scheduleTask(Runnable r, long milliseconds) {
        executor.execute(() -> {
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                return;
            }
            r.run();
        });
    }

    public void stream() {
        webcam.start();
    }

    public void log() {
        if (telemetry != null) {
            telemetry.addData("distance", sensor.getDistance());
            telemetry.addData("Arm Position", arm.getDegree());

            Pose2d poseEstimate = PoseStorage.currentPose;
            telemetry.addData("State", currentState);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void teleOp(Gamepad driverOp, Gamepad toolOp) {
        if (driverOp.x) {
            intake();
        }

        if (driverOp.b) {
            intake.setState(Intake.State.OFF);
        }

        if (driverOp.y) {
            arm.dump();
        }

        if (driverOp.left_bumper) {
            arm.openArm();
        }

        if (driverOp.right_bumper) {
            arm.closeArm();
        }

        if (driverOp.a) {
            dumpy.intake();
            intake.setState(Intake.State.OUTTAKE);
//            scheduleTask(dumpy::close, 150);
        }

        if (toolOp.x) {
            intake.setState(Intake.State.INTAKE);
            dumpy.setState(Dumpy.State.OPEN);
        }

        if (toolOp.b) {
            intake.setState(Intake.State.OFF);
        }
        if (toolOp.a) {
            dumpy.intake();
            intake.setState(Intake.State.OUTTAKE);
//            scheduleTask(dumpy::close, 150);
        }
        if (toolOp.y) {
            fix();
        }


        if (toolOp.left_bumper) {
            arm.openArm();
        }

        if (toolOp.right_bumper) {
            arm.dump();
        }

        if (driverOp.dpad_down) {
            toIntake();
        }

        if (driverOp.dpad_up) {
            toHub();
        }

        if (driverOp.dpad_left) {
            toHubLeft();
        }

        if (driverOp.dpad_right) {
            toHubRight();
        }

        drive(driverOp);
    }

    public void update(Gamepad driverOp, Gamepad toolOp) {
        switch (currentState) {
            case DRIVER_CONTROL: {
                teleOp(driverOp, toolOp);
                break;
            }
            case AUTO_CONTROL: {
                if (driverOp.b) {
                    drive.breakFollowing();
                    currentState = State.DRIVER_CONTROL;
                    break;
                }

                if (!drive.isBusy()) {
                    currentState = State.DRIVER_CONTROL;
                }
                break;
            }
            default: {
                break;
            }
        }

        PoseStorage.currentPose = drive.getPoseEstimate();

        drive.update();
        arm.update();
        secondArm.update();
        armDumpy.update();
        dumpy.update();
        intake.update();

        log();
    }

    public void updateAuto() {
        PoseStorage.currentPose = drive.getPoseEstimate();
        drive.update();
        arm.update();
        secondArm.update();
        armDumpy.update();
        dumpy.update();
        intake.update();

        log();
    }

    public void drive(Gamepad g) {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -g.left_stick_y,
                        -g.left_stick_x,
                        -g.right_stick_x
                )
        );
    }

    public void toIntake() {
        Trajectory intake = drive.trajectoryBuilder(PoseStorage.currentPose)
                .splineTo(new Vector2d(25, 64.65), toRadians(0))
                .lineTo(new Vector2d(60, 64.75))
                .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTemporalMarker(() -> {
                    System.out.println("arm down");
                })
                .setReversed(true)
                .addTrajectory(intake)
                .build();

        currentState = State.AUTO_CONTROL;
        drive.followTrajectorySequenceAsync(toIntake);
    }

    public void toHub() {
        Trajectory hub = drive.trajectoryBuilder(PoseStorage.currentPose)
                .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                .lineTo(new Vector2d(30, 64.75))
                .splineTo(new Vector2d(-13, 40), toRadians(270))
                .build();

        TrajectorySequence toHub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTrajectory(hub)
                .build();

        currentState = State.AUTO_CONTROL;
        drive.followTrajectorySequenceAsync(toHub);
    }

    public void toHubRight() {
        Trajectory hubRight = drive.trajectoryBuilder(PoseStorage.currentPose)
                .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                .lineTo(new Vector2d(20, 64.75))
                .splineTo(new Vector2d(8, 42.8), toRadians(270))
                .lineToSplineHeading(new Pose2d(8, 22.8, toRadians(0)))
                .build();

        TrajectorySequence toHub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTrajectory(hubRight)
                .build();

        currentState = State.AUTO_CONTROL;
        drive.followTrajectorySequenceAsync(toHub);
    }

    public void toHubLeft() {
        Trajectory hubLeft = drive.trajectoryBuilder(PoseStorage.currentPose)
                .lineToSplineHeading(new Pose2d(40, 64.75, toRadians(0)))
                .lineTo(new Vector2d(20, 64.75))
                .splineTo(new Vector2d(-31.5, 42.8), toRadians(270))
                .lineToSplineHeading(new Pose2d(-31.5, 22.8, toRadians(180)))
                .build();

        TrajectorySequence toHub = drive.trajectorySequenceBuilder(PoseStorage.currentPose)
                .addTrajectory(hubLeft)
                .build();

        currentState = State.AUTO_CONTROL;
        drive.followTrajectorySequenceAsync(toHub);
    }


    public void intake() {
        intake.setState(Intake.State.INTAKE);
        dumpy.setState(Dumpy.State.OPEN);
    }

    public void stopIntake() {
        dumpy.close();
        scheduleTask(() -> {
            intake.setState(Intake.State.OUTTAKE);
            scheduleTask(() -> {
                intake.setState(Intake.State.OFF);
            }, 300);

        }, 100);
    }

    public void fix() {
        intake.setState(Intake.State.FIX);
        dumpy.setState(Dumpy.State.OPEN);
        scheduleTask(() -> {
            intake.setState(Intake.State.INTAKE);
        }, 200);
    }

    public void autoIntake() {
        intake.setState(Intake.State.WAIT_FOR_ARM);
        dumpy.setState(Dumpy.State.OPEN);
    }

}
