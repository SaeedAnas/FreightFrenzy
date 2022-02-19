package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.SecondArm;

@Config
@Autonomous(group = "drive")
public class SecondArmTuner extends LinearOpMode {
    SecondArm arm;

    // Highest Point = 0.73
    // up point = 0.65
    // Intake Point = 0.15
    public static double leftPosArm = 0;

    // up point = 0.723
    // Intake Point = 0.223
    public static double rightPosArm = 0.275;

    // PivotServoLeft = 0 // Second Arm Left "secondArmL"
    // PivotServoRight = 1 // Second Arm Right "secondArmR"
    // ArmServoLeft = 2 // ArmDumpy Left "leftArmDumpy"
    // ArmServoRight = 3 // ArmDumpy Right "rightArmDumpy"
    // Dumper = 5 // Dumpy "wall"

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new SecondArm(hardwareMap, null);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            arm.toPos(leftPosArm, rightPosArm);
            telemetry.addData("left", arm.left.getPosition());
            telemetry.addData("right", arm.right.getPosition());
            telemetry.update();

        }


    }
}
