package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.Extensions.cubeInput;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

@Config
public class Drive extends SubsystemBase {

    DcMotorEx lf;
    DcMotorEx lb;
    DcMotorEx rf;
    DcMotorEx rb;

    public static double mult = 0.5;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public enum State {
        DRIVE,
        OFF;
    }

    private State currentState;

    public Drive(HardwareMap hardwareMap) {
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

        currentState = State.DRIVE;
    }

    public void slow() {
        mult = 0.3;
    }

    public void normal() {
        mult = 0.5;
    }

    public void drive(double x, double y, double rx) {
        x = (Math.abs(x) > 0.05) ? x * 1.1 : 0.0; // Counteract imperfect strafing
        y = (Math.abs(y) > 0.05) ?  -y : 0.0; // Remember, this is reversed!
        rx = (Math.abs(rx) > 0.05) ? rx : 0.0;

        x = cubeInput(x, 0.52);
        y = cubeInput(y, 0.52);
        rx = cubeInput(rx, 0.6);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lf.setPower(frontLeftPower * mult);
        lb.setPower(backLeftPower * mult);
        rf.setPower(frontRightPower * mult);
        rb.setPower(backRightPower * mult);

    }

    public void setState(State s) {
        currentState = s;
    }

    public void drive(Gamepad gamepad) {
        switch (currentState) {
            case DRIVE: {
                drive(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);
                break;
            }
        }

    }
}
