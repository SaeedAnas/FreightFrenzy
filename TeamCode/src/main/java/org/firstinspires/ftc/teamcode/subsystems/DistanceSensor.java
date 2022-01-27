package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistanceSensor {
    private final Rev2mDistanceSensor sensor;

    public static double intakeDistance = 10.5;

    public DistanceSensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "intakeSensor");
        sensor.initialize();
    }

    public double getDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public boolean detectBlock() {
        double distance = sensor.getDistance(DistanceUnit.CM);
        return distance <= intakeDistance;
    }
}
