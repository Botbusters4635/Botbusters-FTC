package org.firstinspires.ftc.teamcode.Core

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Controller {
    protected lateinit var telemetry: Telemetry
    protected lateinit var hardwareMap: HardwareMap

    fun init(telemetry: Telemetry, hardwareMap: HardwareMap){
        this.telemetry = telemetry
        this.hardwareMap = hardwareMap
    }
}