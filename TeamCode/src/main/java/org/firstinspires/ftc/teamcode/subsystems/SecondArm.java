package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class SecondArm {
    public Servo left;
    public Servo right;

    private Robot ref;

    public static double diff = 0.275;

    public static double intakePosL = 0;
    public static double intakePosR = intakePosL + diff;

    public static double extendPosL = 0.1;
    public static double extendPosR = extendPosL + diff;

    public static double outtakePosL = 0.5;
    public static double outtakePosR = outtakePosL + diff;


    public enum State {
        INTAKE(intakePosL, intakePosR),
        EXTEND(extendPosL, extendPosR),
        OUTTAKE(outtakePosL, outtakePosR);

        public double leftPos;
        public double rightPos;

        State(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    public State currentState;

    public SecondArm(HardwareMap hardwareMap, Robot robot) {
        left = hardwareMap.get(Servo.class, "secondArmL");
        right = hardwareMap.get(Servo.class, "secondArmR");

        right.setDirection(Servo.Direction.REVERSE);

        ref = robot;

        currentState = State.INTAKE;
    }

    public void intake() {
        setState(State.INTAKE);
        ref.armDumpy.intake();
        ref.dumpy.close();
    }

    public void outtake() {
        setState(State.EXTEND);
        ref.armDumpy.extend();
        ref.dumpy.close();
    }

    public void dump() {
        ref.dumpy.open();
        ref.scheduleTask(() -> {
            ref.dumpy.close();
        }, 600);
    }

    public void high() {
        setState(State.EXTEND);
        ref.armDumpy.high();
        ref.dumpy.close();
    }

    public void outtakeAndDump() {
        setState(State.INTAKE);
        ref.armDumpy.outtake();
        ref.scheduleTask(() -> {
            ref.dumpy.open();
        }, 200);
    }

    public void toPos(double leftPos, double rightPos) {
        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }

    public void setState(State s) {
        currentState = s;
    }

    public void update() {
        switch (currentState) {
            default: {
                toPos(currentState.leftPos, currentState.rightPos);
            }
        }
    }

}
