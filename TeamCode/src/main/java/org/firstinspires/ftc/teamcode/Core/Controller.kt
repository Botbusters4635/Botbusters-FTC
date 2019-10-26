package org.firstinspires.ftc.teamcode.Core

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Controller {
    lateinit var telemetry: Telemetry

    abstract fun init(hardwareMap: HardwareMap)
}