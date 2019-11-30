package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Controller {
    lateinit var telemetry: Telemetry

    abstract fun init(hardwareMap: HardwareMap)

    open fun start() {}

    abstract fun update(timeStep: Double)

    open fun stop() {}
}