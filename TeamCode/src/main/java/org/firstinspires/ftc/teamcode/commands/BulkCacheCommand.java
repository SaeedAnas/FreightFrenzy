package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkCacheCommand extends CommandBase {

    private final List<LynxModule> allHubs;

    public BulkCacheCommand(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
    }
}

