package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Math.toRadians;

import java.util.ArrayList;

@Config
public class Arm extends SubsystemBase {
    private DcMotorEx top;
    private DcMotorEx bottom;

    private Robot ref;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public enum State {
        TOP(410),
        MIDDLE(550),
        BOTTOM(650),
        INTAKE(100),
        OFF(0);

        double position;
        State(double position) {
            this.position = position;
        }
    }

    public State currentState;

    double delta = 25;

    public static double kP = 0.0018;
    public static double kI = 0;
    public static double kD = 0;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    public Arm(HardwareMap hardwareMap, Robot robot) {
        top = hardwareMap.get(DcMotorEx.class, "topArm");
        bottom = hardwareMap.get(DcMotorEx.class, "bottomArm");

        motors.add(top);
        motors.add(bottom);

        motors.forEach((motor) -> {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

        ref = robot;

        currentState = State.OFF;
    }

    public void setPower(double power) {
        motors.forEach((motor) -> {
            motor.setPower(power);
        });
    }

    public void resetEncoders() {
        motors.forEach((motor) -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    public void setState(State s) {
       controller.setTargetPosition(s.position);
       currentState = s;
    }

    public void run() {
        switch (currentState) {
            case OFF: {
                setPower(0);
                resetEncoders();
                break;
            }
            case INTAKE: {
                double error = currentState.position - getAverage();
                if (Math.abs(error) < delta) {
                    setState(State.OFF);
                    break;
                }
            }
            default: {
                double power = controller.update(getAverage());
                setPower(power);
            }
        }
    }

    public double getAverage() {
        double encoder = top.getCurrentPosition() + bottom.getCurrentPosition();
        return encoder / 2;
    }
}
