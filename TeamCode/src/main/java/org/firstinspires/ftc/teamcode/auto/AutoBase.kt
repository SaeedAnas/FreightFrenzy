package org.firstinspires.ftc.teamcode.auto

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.WaitCommand

class AutoBase : CommandOpMode() {

    fun waitFor(millis: Long): WaitCommand = WaitCommand(millis);

    override fun initialize() {
    }

}