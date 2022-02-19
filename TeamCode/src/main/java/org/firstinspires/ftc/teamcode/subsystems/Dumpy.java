package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Dumpy {
    public Servo wall;

    public static long delay = 250;

    private Robot ref;

    // 0.5 up
    // 0 down

    // 0.37 max open
    // 0.25 open

    // 0.075 max closed

    public static double open = 0.25;
    public static double closed = 0.075;

    public enum State {
        OPEN(open),
        CLOSED(closed);

        public double servoPos;

        State(double servoPos) {
            this.servoPos = servoPos;
        }

    }

    public State currentState;

    public Dumpy(HardwareMap hardwareMap, Robot robot) {
        wall = hardwareMap.get(Servo.class, "wall");

        ref = robot;

        currentState = State.CLOSED;
    }

    public void toPos(double pos) {
        wall.setPosition(pos);
    }

    public void setState(State s) {
        currentState = s;
    }

    public void close() {
        setState(Dumpy.State.CLOSED);
    }

    public void intake() {
        setState(State.OPEN);
    }

    public void outtake() {
        setState(State.OPEN);
    }

    public void open() {
        setState(State.OPEN);
    }


    public void update() {
        switch (currentState) {
            default: {
                toPos(currentState.servoPos);
            }
        }
    }
}
