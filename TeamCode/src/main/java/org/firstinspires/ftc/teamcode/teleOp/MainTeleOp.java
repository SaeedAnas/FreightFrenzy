package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.Extensions.cubeInput;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {
    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;
    CRServo carousel;

    double slowModeMult = 1.0;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        // 0 -> lb
        // 1 -> rb
        // 2 -> lf
        // 3 -> rf

        // 0 -> intakeLeft
        // 1 -> intakeRight
        // 2 -> topArm -- ENCODER ARM
        // 3 -> bottomArm

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        carousel = hardwareMap.get(CRServo.class, "carousel");

        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
//        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }); // Sets the power decrease of Run using encoder to 0%, making the max speed back to 100%;

        waitForStart();

        while (opModeIsActive()) {
            if(isStopRequested()) return;


            double y = (Math.abs(gamepad1.left_stick_y) > 0.05) ?  -gamepad1.left_stick_y : 0.0; // Remember, this is reversed!
            double x = (Math.abs(gamepad1.left_stick_x) > 0.05) ? gamepad1.left_stick_x * 1.1 : 0.0; // Counteract imperfect strafing
            double rx = (Math.abs(gamepad1.right_stick_x) > 0.05) ? gamepad1.right_stick_x : 0.0;

            y = cubeInput(y, 0.52);
            x = cubeInput(x, 0.52);
            rx = cubeInput(rx, 0.6);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            slowModeMult = gamepad1.left_bumper ? 0.5 : 1;

            lf.setPower(frontLeftPower * slowModeMult);
            lb.setPower(backLeftPower * slowModeMult);
            rf.setPower(frontRightPower * slowModeMult);
            rb.setPower(backRightPower * slowModeMult);

            if (gamepad1.a) {
                carousel.set(1);
            } else {
                carousel.set(0);
            }

        }

    }
}
