package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {

    private DcMotorSimple leftIntake;
    private DcMotorSimple rightIntake;
    private Telemetry telemetry;
    private State state = State.STOP;

    public enum State {
        INTAKE(0.75),
        OUTTAKE(-0.75),
        STOP(0);

        double power;
        State(double power) {
            this.power = power;
        }
    }

    public Intake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorSimple.class, "rightIntake");
    }

    public void setState(State state) {
        leftIntake.setPower(state.power);
        rightIntake.setPower(state.power);
        this.state = state;
    }

    public void intake() {
        setState(State.INTAKE);
    }

    public void outtake() {
        setState(State.OUTTAKE);
    }

    public void stop() {
        setState(State.STOP);
    }

}
