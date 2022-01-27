package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Intake extends SubsystemBase {

    public DcMotorEx intakeLeft;
    public DcMotorEx intakeRight;
    Robot ref;

    private State currentState;

    public static double runningVelo = 1480;
    public static double delta = 500;

    public enum State {
        RAMP(0.75),
        INTAKE(0.75),
        INTAKING(0.75),
        OUTTAKE(-0.75),
        WAIT(0),
        OFF(0);

        double power;
        State(double power) {
            this.power = power;
        }
    }

    public Intake(HardwareMap hardwareMap, Robot robot) {
        intakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

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

    public void run() {

        switch (currentState) {
            case RAMP: {
                double error = runningVelo - getVelocity();
                if (Math.abs(error) < delta) {
                    setState(State.INTAKE);
                }
                setPower(currentState.power);
            }
            case INTAKE: {
                double error = runningVelo - getVelocity();
                if (Math.abs(error) > delta) {
                    setState(State.INTAKING);
                }
                setPower(currentState.power);
                break;
            }
            case WAIT: {
                break;
            }
            case INTAKING: {
                double error = runningVelo - getVelocity();
                if (Math.abs(error) < delta) {
                    setState(State.WAIT);
                    ref.scheduleTask(() -> {
                        setState(State.OFF);
                        ref.dumpy.close();
                    }, 250);
                    break;
                }
                setPower(currentState.power);
                break;
            }
            default: {
                setPower(currentState.power);
            }

        }
    }

}
