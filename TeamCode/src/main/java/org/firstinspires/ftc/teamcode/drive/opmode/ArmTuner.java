package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")
public class ArmTuner extends LinearOpMode {
    DcMotorEx ta;
    DcMotorEx ba;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    public static double reference = 400;

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

        PIDCoefficients coeffs = new PIDCoefficients(Kp, Ki, Kd);

        PIDFController controller = new PIDFController(coeffs);

        controller.setTargetPosition(reference);


        ta = hardwareMap.get(DcMotorEx.class, "topArm");
        ba = hardwareMap.get(DcMotorEx.class, "bottomArm");

        motors.add(ta);
        motors.add(ba);

        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }); // Sets the power decrease of Run using encoder to 0%, making the max speed back to 100%;

        ta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));


        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("ta", ta.getCurrentPosition());
            telemetry.addData("reference", reference);

            double power = controller.update(ta.getCurrentPosition());

            telemetry.addData("power", power);

            ta.setPower(power);
            ba.setPower(power);

//            if (gamepad1.a) {
//                ta.setTargetPosition(400);
//                ta.setPower(power);
//                ba.setPower(power);
//            } else {
//                ta.setPower(0);
//                ba.setPower(0);
//            }
//
//            if (gamepad1.b) {
//                ta.setPower(-power);
//                ba.setPower(-power);
//            } else {
//                ta.setPower(0);
//                ba.setPower(0);
//            }


            telemetry.update();

        }

    }

//    public double PIDControl(double refernce, double state) {
//        double error = refernce - state;
//        double errorChange = error - lastError;
//
//        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
//        previousFilterEstimate = currentFilterEstimate;
//
//        double derivative = currentFilterEstimate / timer.seconds();
//
//        integralSum += error * timer.seconds();
//
//        if (integralSum > maxIntegralSum) {
//            integralSum = maxIntegralSum;
//        }
//
//        if (integralSum < -maxIntegralSum) {
//            integralSum = -maxIntegralSum;
//        }
//
//        if (refernce != lastReference) {
//            integralSum = 0;
//        }
//
//
//        lastError = error;
//
//        lastReference = refernce;
//        timer.reset();
//
//        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//        return output;
//    }
}
