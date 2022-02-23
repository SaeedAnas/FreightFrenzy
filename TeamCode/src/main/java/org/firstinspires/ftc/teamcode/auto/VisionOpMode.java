package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.vision.ColorFilterPipeline;

@Config
@Autonomous(group = "blue")
public class VisionOpMode extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.stream();

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            double pos = ColorFilterPipeline.xPos;
            double width = ColorFilterPipeline.boxWidth;
            telemetry.addData("xPos", pos);
            telemetry.addData("width", width);

            if (pos > 400 && width > 100) {
                telemetry.addData("Level", "TOP");
            } else if (pos < 400 && pos > 100 && width > 100) {
                telemetry.addData("Level", "MIDDLE");
            } else {
                telemetry.addData("Level", "BOTTOM");
            }

//            telemetry.addData("xPos", ColorFilterPipeline.xPos);
            telemetry.update();

        }
    }

}

