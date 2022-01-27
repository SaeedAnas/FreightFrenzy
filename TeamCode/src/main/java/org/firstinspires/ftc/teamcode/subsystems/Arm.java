package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        INTAKE(65),
        PROFILE(410),
        OFF(0);

        double position;

        State(double position) {
            this.position = position;
        }
    }

    public State currentState;

    public MotionProfile currentProfile;

    double delta = 25;

    public static double kP = 0.0018;
    public static double kI = 0;
    public static double kD = 0;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    ElapsedTime t = new ElapsedTime();

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

        currentProfile = null;
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

    public void closeArm() {
        ref.dumpy.close();
        ref.scheduleTask(() -> {
            setState(State.INTAKE);
        }, 150);
    }

    public void openArm() {
        ref.dumpy.close();
        ref.scheduleTask(() -> {
            setState(State.TOP);
        }, 150);
    }

    public void testProfile() {
        currentProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0,0), new MotionState(410, 0,0), 10, 10);
        t.reset();
        setState(State.PROFILE);
    }

    public void run() {
        switch (currentState) {
            case OFF: {
                setPower(0);
                resetEncoders();
                break;
            }
            case INTAKE: {
                if (hasReached()) {
                    setState(State.OFF);
                    break;
                }
            }
            case PROFILE: {
                MotionState state = currentProfile.get(t.milliseconds());
                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());
                double power = controller.update(getAverage());
                setPower(power);
            }
            default: {
                double power = controller.update(getAverage());
                setPower(power);
            }
        }
    }

    public boolean hasReached() {
        double error = currentState.position - getAverage();
        return Math.abs(error) < delta;
    }

    public double getAverage() {
        double encoder = top.getCurrentPosition() + bottom.getCurrentPosition();
        return encoder / 2;
    }
}
