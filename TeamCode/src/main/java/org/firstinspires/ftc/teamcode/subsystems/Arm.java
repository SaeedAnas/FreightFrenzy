package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.toRadians;

import java.util.ArrayList;

@Config
public class Arm extends SubsystemBase {
    private DcMotorEx top;
    private DcMotorEx bottom;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public enum State {
        TOP(450),
        MIDDLE(550),
        BOTTOM(650),
        INTAKE(10);

        double position;
        State(double position) {
            this.position = position;
        }
    }

    public State currentState = State.INTAKE;

    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    public Arm(HardwareMap hardwareMap) {
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

        setState(currentState);
    }

    public void setPower(double power) {
        motors.forEach((motor) -> {
            motor.setPower(power);
        });
    }

    public int getPos() {
        return top.getCurrentPosition();
    }

    public double getPower() {
        return controller.update(getPos());
    }

    public void setState(State s) {
       controller.setTargetPosition(s.position);
    }

    public void toPos() {
        double power = getPower();
        setPower(power);
    }

    public void toTop() {
        setState(State.TOP);
        toPos();
    }

    public void toMiddle() {
        setState(State.MIDDLE);
        toPos();
    }

    public void toBottom() {
        setState(State.BOTTOM);
        toPos();
    }

    public void toIntake() {
        setState(State.INTAKE);
        toPos();
    }

    public void off() {
        setPower(0);
    }
}
