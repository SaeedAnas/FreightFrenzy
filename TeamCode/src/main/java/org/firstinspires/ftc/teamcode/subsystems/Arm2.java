package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@Config
public class Arm2 extends SubsystemBase {
    private DcMotorEx top;
    private DcMotorEx bottom;

    private Robot ref;


    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public State currentState;
    public MotionProfile currentProfile;

    public static double delta = 100;

    public static double kP = 0.002;
    public static double kI = 0;
    public static double kD = 0;

    public static double v = 0.5;
    public static double a = 0.5;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    ElapsedTime t = new ElapsedTime();

    public enum State {
        TOP(410, v, a),
        MIDDLE(535, v, a),
        BOTTOM(650, v, a),
        OFF(0, v, a),
        INTAKE(65, 0.2,0.2);

        double position;
        double velo;
        double accel;

        State(double position, double velo, double accel) {
            this.position = position;
            this.velo = velo;
            this.accel = accel;
        }
    }

    public Arm2(HardwareMap hardwareMap, Robot robot) {
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
        currentState = s;
        currentProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getAverage(), 0, 0), new MotionState(currentState.position, 0, 0), currentState.velo, currentState.accel);
    }

    public boolean hasReached() {
        double error = currentState.position - getAverage();
        return Math.abs(error) < delta;
    }

    public double getAverage() {
        double encoder = top.getCurrentPosition() + bottom.getCurrentPosition();
        return encoder / 2;
    }

    public void closeArm() {
        ref.dumpy.intake();
        ref.scheduleTask(() -> {
            setState(State.INTAKE);
        }, 250);
    }

    public void openArm(State s) {
        ref.dumpy.close();
        ref.scheduleTask(() -> {
            setState(s);
        }, 150);
    }

    public void top() {
        openArm(State.TOP);
    }

    public void middle() {
        openArm(State.MIDDLE);
    }

    public void bottom() {
        openArm(State.BOTTOM);
    }

    public void dump() {
        ref.dumpy.outtake();
        ref.scheduleTask(this::closeArm, 1000);
    }

    public void powerDump() {
        ref.dumpy.powerOuttake();
        ref.scheduleTask(this::closeArm, 1000);
    }


    public void run() {
        switch(currentState) {
            case OFF: {
                setPower(0);
                resetEncoders();
                break;
            }
            case INTAKE: {
                if(hasReached()) {
                    setState(State.OFF);
                }
            }
            case TOP:
            case MIDDLE:
            case BOTTOM: {
                ref.drive.slow();
            }
            default: {
                MotionState state = currentProfile.get(t.milliseconds());
                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());
                double power = controller.update(getAverage());
                setPower(power);
            }
        }
    }


}
