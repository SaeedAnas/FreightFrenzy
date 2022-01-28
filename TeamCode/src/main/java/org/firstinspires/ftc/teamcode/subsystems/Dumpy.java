package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.Timer;
import java.util.TimerTask;

@Config
public class Dumpy {
    public Servo top;
    public Servo bottom;

    public static long delay = 250;

    private Robot ref;

//    public static double topOpen = 0.5;
    public static double topOpen = 0.07;
//    public static double topClosed = 0.65;
    public static double topClosed = 0.2;
//    public static double topCloser = 0.69;
    public static double topCloser = 0.2;
    //    public static double topPush = 0.75;
    public static double topPush = 0.3;
//    public static double topPower = 0.9;
    public static double topPower = 0.3;

    public static double bottomClosed = 0.3;
    public static double bottomOpen = 0.15;

    public enum State {
        INTAKE(topOpen, bottomClosed),
        CLOSED(topClosed,bottomClosed),
        OPEN(topCloser, bottomClosed),
        WAIT(0,0),
        POWER(topPower, bottomOpen),
        OUTTAKE(topPush,bottomOpen);

        public double topPos;
        public double bottomPos;

        State(double topPos, double bottomPos) {
            this.topPos = topPos;
            this.bottomPos = bottomPos;
        }

    }

    public State currentState;

    public Dumpy(HardwareMap hardwareMap, Robot robot) {
        top = hardwareMap.get(Servo.class, "top");
        bottom = hardwareMap.get(Servo.class, "bottom");

        ref = robot;

        currentState = State.INTAKE;
    }

    public void toPos(double topPos, double bottomPos) {
        top.setPosition(topPos);
        bottom.setPosition(bottomPos);
    }

    public void setState(State s) {
        currentState = s;
    }

    public void close() {
        setState(Dumpy.State.CLOSED);
    }

    public void intake() {
        setState(State.INTAKE);
    }

    public void outtake() {
        setState(State.OUTTAKE);
    }

    public void powerOuttake() {
        setState(State.POWER);
    }

    public void open() {
        setState(State.OPEN);
    }


    public void run() {
        switch (currentState) {
            case OUTTAKE: {
                bottom.setPosition(currentState.bottomPos);
                ref.scheduleTask(() -> top.setPosition(topPush), 150);
                setState(State.WAIT);
                break;
            }
            case POWER: {
                bottom.setPosition(currentState.bottomPos);
                ref.scheduleTask(() -> top.setPosition(topPower), 150);
                setState(State.WAIT);
                break;

            }
            case WAIT: {
                break;
            }
            default: {
                toPos(currentState.topPos, currentState.bottomPos);
                setState(State.WAIT);
            }
        }
    }
}
