package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class ArmDumpy {
    public Servo left;
    public Servo right;

    private Robot ref;

    public static double intakePosL = 0.185;
    public static double intakePosR = 0.185;

    public static double outtakePosL = 0.63;
    public static double outtakePosR = 0.63;

    public static double extendPosL = 0.75;
    public static double extendPosR = 0.75;

    public static double highPosL = 0.8;
    public static double highPosR = 0.8;


    public enum State {
        INTAKE(intakePosL, intakePosR),
        OUTTAKE(outtakePosL, outtakePosR),
        EXTEND(extendPosL, extendPosR),
        MIDDLE(0.65, 0.65),
        BOTTOM(0.5, 0.5),
        HIGH(highPosL, highPosR);

        public double leftPos;
        public double rightPos;

        State(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    public State currentState;

    public ArmDumpy(HardwareMap hardwareMap, Robot ref) {
        left = hardwareMap.get(Servo.class, "leftArmDumpy");
        right = hardwareMap.get(Servo.class, "rightArmDumpy");

        left.setDirection(Servo.Direction.REVERSE);

        this.ref = ref;

        currentState = State.INTAKE;
    }

    public void intake() {
        setState(State.INTAKE);
    }

    public void outtake() {
        setState(State.OUTTAKE);
    }

    public void extend() {
        setState(State.EXTEND);
    }

    public void high() {
        setState(State.HIGH);
    }

    public void middle() {
        setState(State.MIDDLE);
    }

    public void bottom() {
        setState(State.BOTTOM);
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
