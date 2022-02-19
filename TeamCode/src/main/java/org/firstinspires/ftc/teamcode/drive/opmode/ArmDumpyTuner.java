package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmDumpy;

@Config
@Autonomous(group = "drive")
public class ArmDumpyTuner extends LinearOpMode {
    ArmDumpy arm;

    // 0 is intake position
    public static double leftPosArm = 0;
    public static double rightPosArm = 0;

    // PivotServoLeft = 0 // Second Arm Left "secondArmL"
    // PivotServoRight = 1 // Second Arm Right "secondArmR"
    // ArmServoLeft = 2 // ArmDumpy Left "leftArmDumpy"
    // ArmServoRight = 3 // ArmDumpy Right "rightArmDumpy"
    // Dumper = 5 // Dumpy "wall"

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new ArmDumpy(hardwareMap, null);

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            arm.toPos(leftPosArm, rightPosArm);

        }


    }
}
