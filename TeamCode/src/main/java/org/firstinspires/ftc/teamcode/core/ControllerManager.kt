package org.firstinspires.ftc.teamcode.core

import android.os.SystemClock
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class ControllerManager {
    constructor()
    constructor(telemetry: Telemetry, hardwareMap: HardwareMap) {
        this.hardwareMap = hardwareMap
        this.telemetry = telemetry
    }

    private val controllers: ArrayList<Controller> = arrayListOf()
    private val lastTimeRun: ArrayList<Double> = arrayListOf(0.0)
    lateinit var telemetry: Telemetry
    lateinit var hardwareMap: HardwareMap

    fun add(controller: Controller) {
        controllers.add(controller)
        lastTimeRun.add(0.0)
    }

    fun start() {
        controllers.withIndex().forEach {
            it.value.start()
            lastTimeRun[it.index] = SystemClock.elapsedRealtime() / 1000.0
            it.value.controllers.start()
        }
    }

    fun update() {
        controllers.withIndex().forEach {
            val timeStep = SystemClock.elapsedRealtime() / 1000.0 - lastTimeRun[it.index]
            lastTimeRun[it.index] = SystemClock.elapsedRealtime().toDouble() / 1000.0
            it.value.update(timeStep)
            it.value.controllers.update()
        }
    }

    fun stop() {
        controllers.forEach {
            it.controllers.stop()
            it.stop()
        }
    }

    fun init() {
        controllers.forEach {
            it.controllers = ControllerManager(telemetry, hardwareMap)
            it.telemetry = telemetry
            it.init(hardwareMap)
            it.controllers.init()
        }
    }
}