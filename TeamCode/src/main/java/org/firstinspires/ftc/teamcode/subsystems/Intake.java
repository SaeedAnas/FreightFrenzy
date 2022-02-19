package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Intake extends SubsystemBase {

    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    Robot ref;

    private State currentState;

    public static double p = -0.5;

    public enum State {
        INTAKE(p),
        OUTTAKE(-p),
        FIX(p),
        ARM_UP(-0.3),
        ARM_DOWN(0.3),
        WAIT(0),
        WAIT_FOR_ARM(0),
        OFF(0);

        double power;

        State(double power) {
            this.power = power;
        }
    }

    public Intake(HardwareMap hardwareMap, Robot robot) {
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        currentState = State.OFF;
        ref = robot;
    }

    public void setPower(double p) {
        intakeLeft.setPower(p);
        intakeRight.setPower(p);
    }

    public void setState(State state) {
        currentState = state;
    }

    public double getVelocity() {
        return intakeLeft.getVelocity();
    }

    public void update() {

        switch (currentState) {
            case INTAKE: {
                setPower(currentState.power);
                if (intakeLeft.isOverCurrent()) {
                    setState(State.OUTTAKE);
                    ref.scheduleTask(() -> {
                        setState(State.OFF);
                    }, 200);
                }
                if (ref.sensor.detectBlock()) {
                    setState(State.OUTTAKE);
                    ref.scheduleTask(() -> setState(State.OFF), 200);
                    ref.dumpy.close();
                }
                break;
            }
            case WAIT: {
                break;
            }
            case WAIT_FOR_ARM: {
                if (ref.arm.currentState == Arm.State.OFF) {
                    ref.scheduleTask(() -> {
                        setState(State.INTAKE);
                    }, 1000);
                }
                break;
            }
            default: {
                setPower(currentState.power);
            }

        }
    }

    public boolean hasFreight() {
        return currentState == State.OFF;
    }
}
