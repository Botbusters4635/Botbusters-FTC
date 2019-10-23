package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Controller {
    lateinit var telemetry: Telemetry
    lateinit var hardwareMap: HardwareMap
}

