package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.toRadians;

public class Arm extends SubsystemBase {
    private DcMotorEx top;
    private DcMotorEx bottom;

    public static PIDCoefficients coefficients = new PIDCoefficients(3.0, 5.0, 0.5);
    public static double INTEGRAL_BAND = toRadians(2.0);
    public static double TOLERANCE = toRadians(0.5);

    public static double MIN_ANGLE = Math.toRadians(0);
    public static double MAX_ANGLE = Math.toRadians(150);

    public static Vector2d ARM_OFFSET = new Vector2d(0, 0);

    public static final double TICKS_PER_REF = 28 * 50.9;
    public static final double GEAR_RATIO = 20.0 / 24.0;

    public static double target;
    public static double kStatic = 0.04;
    private double lastPosition;


    private Telemetry telemetry;

    public enum Positions {
        TOP(0),
        MIDDLE(0),
        BOTTOM(0),
        INSIDE(0);

        int distance;
        Positions(int distance) {this.distance = distance;}
    }

    public enum State {
        MOVING,
        STOPPED
    }

    private State state = State.STOPPED;

    public Arm(HardwareMap hardwareMap) { this(hardwareMap, null);}

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        top = hardwareMap.get(DcMotorEx.class, "topArm");
        bottom = hardwareMap.get(DcMotorEx.class, "bottomArm");

        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetPos();

        this.telemetry = telemetry;

    }

    public void resetPos() {
        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
