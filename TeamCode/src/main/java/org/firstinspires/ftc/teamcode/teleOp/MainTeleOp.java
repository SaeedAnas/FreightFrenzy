package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;


@Config
@TeleOp(name="Main TeleOp", group="TeleOp")
public class MainTeleOp extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // 0 -> lb
        // 1 -> rb
        // 2 -> lf
        // 3 -> rf

        // 0 -> intakeLeft
        // 1 -> intakeRight
        // 2 -> topArm
        // 3 -> bottomArm

        robot = new Robot(hardwareMap, telemetry);

        robot.stream();

        waitForStart();

        while (opModeIsActive()) {
            if(isStopRequested()) return;

            robot.teleOp(gamepad1, gamepad2);
        }

        robot.shutdown();

    }

}
