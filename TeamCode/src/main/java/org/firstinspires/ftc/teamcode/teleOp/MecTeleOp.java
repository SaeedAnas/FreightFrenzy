package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.Extensions.cubeInput;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@TeleOp(name="Mecanum Drive", group="TeleOp")
public class MecTeleOp extends LinearOpMode {
    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        motors.add(lf);
        motors.add(lb);
        motors.add(rf);
        motors.add(rb);

        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        motors.forEach((motor) -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));

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

            double mult = 0.5;

            lf.setPower(frontLeftPower * mult);
            lb.setPower(backLeftPower * mult);
            rf.setPower(frontRightPower * mult);
            rb.setPower(backRightPower * mult);

            telemetry.addData("lf", lf.getCurrentPosition());
            telemetry.addData("lb", lb.getCurrentPosition());
            telemetry.addData("rf", rf.getCurrentPosition());
            telemetry.addData("rb", rb.getCurrentPosition());

            telemetry.update();

        }

    }
}
