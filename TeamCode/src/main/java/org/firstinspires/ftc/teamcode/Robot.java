package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm2;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Dumpy;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.vision.Webcam;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot {
    public Drive drive;
    public Arm arm;
    public Arm2 arm2;
    public Intake intake;
    public Dumpy dumpy;
    public DistanceSensor sensor;
//    public Webcam webcam;
    Telemetry telemetry;

    ExecutorService executor = Executors.newFixedThreadPool(10);

    public enum State {
        INTAKE,
        GRAB,
        DRIVE,
        SCORE,
        AUTO;
    }

    private State currentState;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap, this);
        arm2 = new Arm2(hardwareMap, this);
        intake = new Intake(hardwareMap, this);
        sensor = new DistanceSensor(hardwareMap);
//        webcam = new Webcam(hardwareMap);
        dumpy = new Dumpy(hardwareMap, this);

        this.telemetry = telemetry;

        currentState = State.DRIVE;
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

//    public void stream() {
//        webcam.start();
//    }

    public void log() {
        if (telemetry != null) {

            telemetry.addData("leftamp", intake.intakeLeft.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("rightamp", intake.intakeRight.getCurrent(CurrentUnit.MILLIAMPS));

            telemetry.addData("leftv", intake.intakeLeft.getVelocity());
            telemetry.addData("rightv", intake.intakeRight.getVelocity());

            telemetry.addData("distance",sensor.getDistance());

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

        if (driverOp.left_trigger > 0.5) {
            drive.slow();
        } else {
            drive.normal();
        }

        if (driverOp.a) {
            dumpy.intake();
            intake.setState(Intake.State.OUTTAKE);
//            scheduleTask(dumpy::close, 150);
        }

        if (toolOp.x) {
            intake.setState(Intake.State.INTAKE);
            dumpy.setState(Dumpy.State.INTAKE);
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



        drive.drive(driverOp);
        arm.run();
        intake.run();
        dumpy.run();

        log();
    }

    public void intake() {
        intake.setState(Intake.State.INTAKE);
        dumpy.setState(Dumpy.State.INTAKE);
    }

    public void fix() {
        intake.setState(Intake.State.FIX);
        dumpy.setState(Dumpy.State.INTAKE);
        scheduleTask(() -> {intake.setState(Intake.State.INTAKE);}, 200);
    }
    public void autoIntake() {
        intake.setState(Intake.State.WAIT_FOR_ARM);
        dumpy.setState(Dumpy.State.INTAKE);
    }

    public void autoRun() {
        arm2.run();
        intake.run();
        dumpy.run();
    }

    public void setState(State s) {
        currentState = s;
        switch (currentState) {
            case INTAKE: {
                arm.setState(Arm.State.INTAKE);
                intake.setState(Intake.State.INTAKE);
                break;
            }
        }
    }

}
