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
public class Arm extends SubsystemBase {

    public static double TICKS_PER_REV = 1425.1;
    public static double ZERO_ANGLE = 0;

    private DcMotorEx top;
    private DcMotorEx bottom;

    private Robot ref;

    ArrayList<DcMotorEx> motors = new ArrayList<>();

    public static double topv = 0.18;
    public static double topa = 0.05;

    public static double intakev = 0.18;
    public static double intakea = 0.05;

    public enum State {
        TOP(90, topv, topa),
        MIDDLE(130, intakev, intakea),
        BOTTOM(150, 0.13, 0.05),
        INTAKE(10, 0.08, 0.05),
        PROFILE(90, 0.18, 0.05),
        UP(90, 0.18, 0.05),
        OFF(0, 0.01, 0.01);

        double angle;
        double v;
        double a;

        State(double angle, double v, double a) {
            this.angle = angle;
            this.v = v;
            this.a = a;
        }
    }

    public State currentState;
    public MotionProfile currentProfile;

    private State level = State.TOP;

    public static double delta = 3;

    public static double kP = 0.013;
    public static double kI = 0;
    public static double kD = 0;
    public static double kCos = 0.025;
    public static int dumpCloseTime = 300;
    public static int dumpOpenTime = 1000;

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
        currentProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getDegree(), 0, 0), new MotionState(s.angle, 0, 0), s.v, s.a);
        currentState = s;
        t.reset();
    }

    public void setAngle(double a) {
        controller.setTargetPosition(a);
        currentState = State.TOP;
    }

    public void top() {
        setState(State.TOP);
        ref.intake.setState(Intake.State.ARM_UP);
        ref.scheduleTask(() -> {
            ref.intake.setState(Intake.State.OFF);
        }, 300);
        ref.scheduleTask(() -> {
            ref.secondArm.outtake();
        }, dumpOpenTime);
    }

    public void middle() {
        setState(State.MIDDLE);
        ref.intake.setState(Intake.State.ARM_UP);
        ref.scheduleTask(() -> {
            ref.intake.setState(Intake.State.OFF);
        }, 300);

        ref.scheduleTask(() -> {
            ref.secondArm.outtake();
        }, dumpOpenTime);
    }

    public void bottom() {
        setState(State.BOTTOM);
        ref.intake.setState(Intake.State.ARM_UP);
        ref.scheduleTask(() -> {
            ref.intake.setState(Intake.State.OFF);
        }, 300);
        ref.scheduleTask(() -> {
            ref.secondArm.outtake();
        }, dumpOpenTime);
    }


    public void closeArm() {
        ref.secondArm.intake();
        ref.intake.setState(Intake.State.ARM_DOWN);
        ref.scheduleTask(() -> {
            setState(State.INTAKE);
            ref.scheduleTask(() -> {
                ref.intake.setState(Intake.State.OFF);
            }, 1300);
        }, dumpCloseTime);
    }

    public void openArm() {
        top();
    }

    public void dump() {
        ref.secondArm.dump();
//        ref.scheduleTask(this::closeArm, 1000);
    }

    public void update() {
        switch (currentState) {
            case OFF: {
                setPower(0);
                resetEncoders();
                break;
            }
            case PROFILE: {
                MotionState state = currentProfile.get(t.milliseconds());
                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());
//                double power = controller.update(getDegree()) + (kCos * Math.cos(Math.toRadians(getDegree())));
                double power = controller.update(getDegree());
                setPower(power);
                break;
            }
            case UP: {
                double power = controller.update(getDegree()) + (kCos * Math.cos(Math.toRadians(getDegree())));
                setPower(power);

                if (hasReached()) {
                    ref.dumpy.open();
                    setState(level);
                }
                break;

            }
            case INTAKE: {
                if (hasReached()) {
                    setState(State.OFF);
                    break;
                }
            }
            case TOP:
            case MIDDLE:
            case BOTTOM: {
            }
            default: {
//                double power = controller.update(getDegree()) + (kCos * Math.cos(Math.toRadians(getDegree())));
//                setPower(power);
                MotionState state = currentProfile.get(t.milliseconds());
                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());
//                double power = controller.update(getDegree()) + (kCos * Math.cos(Math.toRadians(getDegree())));
                double power = controller.update(getDegree());
                setPower(power);

            }
        }
    }

    public boolean hasReached() {
        double error = currentState.angle - getDegree();
        return Math.abs(error) < delta;
    }

    public double getDegree() {
        double encoder = top.getCurrentPosition();
        double angle = (encoder / TICKS_PER_REV) * 360;

        return angle + ZERO_ANGLE;
    }
}
