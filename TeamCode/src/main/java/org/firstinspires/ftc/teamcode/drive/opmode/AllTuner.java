package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(group = "drive")
public class AllTuner extends LinearOpMode {
//    ArmDumpy armDumpy;
//    SecondArm secondArm;
//    Dumpy dumpy;
//    Arm bigArm;
//
//    ExecutorService executor = Executors.newFixedThreadPool(10);
//
//    public void shutdown() {
//        executor.shutdown();
//    }
//
//    public void scheduleTask(Runnable r, long milliseconds) {
//        executor.execute(() -> {
//            try {
//                Thread.sleep(milliseconds);
//            } catch (InterruptedException e) {
//                return;
//            }
//            r.run();
//        });
//    }

    Robot robot;

    // 0.185 is intake position
    // 0.63 is outtake position
    public static double leftPosDumpy = 0.185;
    public static double rightPosDumpy = 0.185;


    // Highest Point = 0.73
    // up point = 0.65
    // Intake Point = 0.15
    public static double leftPosArm = 0;

    // up point = 0.723
    // Intake Point = 0.223
    public static double rightPosArm = 0.275;

    // open = 0.25
    // closed = 0.075
    public static double dumpyPos = 0.075;

    public static double angle = 90;
    public static double accel = 0;
    public static double velo = 0;

    // PivotServoLeft = 0 // Second Arm Left "secondArmL"
    // PivotServoRight = 1 // Second Arm Right "secondArmR"
    // ArmServoLeft = 2 // ArmDumpy Left "leftArmDumpy"
    // ArmServoRight = 3 // ArmDumpy Right "rightArmDumpy"
    // Dumper = 5 // Dumpy "wall"

    @Override
    public void runOpMode() throws InterruptedException {
//        armDumpy = new ArmDumpy(hardwareMap, null);
//        secondArm = new SecondArm(hardwareMap, null);
//        dumpy = new Dumpy(hardwareMap, null);
//        bigArm = new Arm(hardwareMap, null);
        robot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        // Angle 90, kP 0.013, velo 0.18, accel 0.05
        // Angle 5, kP 0.013, velo 0.08, accel 0.05
        // Angle 130, kP 0.013, velo 0.13, accel 0.05
//        bigArm.setProfile(angle, velo, accel);

        while (opModeIsActive()) {
            if (isStopRequested()) return;


            if (gamepad1.dpad_up) {
                robot.arm.openArm();
            }
            if (gamepad1.dpad_down) {
                robot.arm.closeArm();
            }
            if (gamepad1.dpad_left) {
                robot.arm.middle();
            }
            if (gamepad1.dpad_right) {
                robot.arm.bottom();
            }

            if (gamepad1.y) {
                robot.secondArm.outtake();
            }

            if (gamepad1.a) {
                robot.secondArm.dump();
            }

            if (gamepad1.b) {
                robot.stopIntake();
            }

            if (gamepad1.x) {
                robot.intake();
            }

            robot.drive(gamepad1);

            robot.updateAuto();


        }


    }
}
