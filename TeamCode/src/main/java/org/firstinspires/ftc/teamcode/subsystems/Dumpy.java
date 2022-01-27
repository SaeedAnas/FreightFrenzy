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

    public enum State {
        INTAKE(0.5, 0),
        CLOSED(0.65,0),
        OPEN(0.65, 0.6),
        WAIT(0,0),
        OUTTAKE(1,0.6);

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


    public void run() {
        switch (currentState) {
            case OUTTAKE: {
                bottom.setPosition(currentState.bottomPos);
                ref.scheduleTask(() -> top.setPosition(currentState.topPos), delay);
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
