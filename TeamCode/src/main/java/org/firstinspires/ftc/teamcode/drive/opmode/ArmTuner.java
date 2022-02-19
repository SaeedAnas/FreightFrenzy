package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
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

    public static double Kp = 0.002;
    public static double Ki = 0.00013;
    public static double Kd = 0;

    public static double reference = 400;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public MotionProfile currentProfile;

    ElapsedTime t = new ElapsedTime();

    public static double velo = 10;
    public static double accel = 10;
    public static double pos = 400;

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

        ba.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.forEach((motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));


        waitForStart();

        currentProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0, 0, 0), new MotionState(pos, 0, 0), velo, accel);
        t.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("ta", ba.getCurrentPosition());
            telemetry.addData("reference", reference);

            MotionState state = currentProfile.get(t.milliseconds());
            controller.setTargetPosition(state.getX());
            controller.setTargetVelocity(state.getV());
            controller.setTargetAcceleration(state.getA());
            double power = controller.update(getAverage());

            ta.setPower(power);
            ba.setPower(power);

            telemetry.update();

        }

    }

    public double getAverage() {
        double encoder = ta.getCurrentPosition() + ba.getCurrentPosition();
        return encoder / 2;
    }

}
