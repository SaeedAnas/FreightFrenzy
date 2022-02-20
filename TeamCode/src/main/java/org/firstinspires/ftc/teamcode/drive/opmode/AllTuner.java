package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(group = "Actual")
public class AllTuner extends LinearOpMode {
    Robot robot;

    // PivotServoLeft = 0 // Second Arm Left "secondArmL"
    // PivotServoRight = 1 // Second Arm Right "secondArmR"
    // ArmServoLeft = 2 // ArmDumpy Left "leftArmDumpy"
    // ArmServoRight = 3 // ArmDumpy Right "rightArmDumpy"
    // Dumper = 5 // Dumpy "wall"

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;


            if (gamepad1.dpad_up) {
                robot.arm.openArm();
            }
            if (gamepad1.right_bumper) {
                robot.arm.openArm();
            }
            if (gamepad1.dpad_down) {
                robot.arm.closeArm();
            }
            if (gamepad1.left_bumper) {
                robot.arm.closeArm();
            }
            if (gamepad1.dpad_left) {
                robot.arm.middle();
            }
            if (gamepad1.dpad_right) {
                robot.intake.setState(Intake.State.CAROUSEL);
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

            if (gamepad1.left_trigger > 0.4) {
                robot.intake();
            }

            if (gamepad1.right_trigger > 0.4) {
                robot.arm.dump();
            }

            robot.driveFieldCentric(gamepad1);
//            robot.log();

            robot.updateAuto();


        }


    }
}
