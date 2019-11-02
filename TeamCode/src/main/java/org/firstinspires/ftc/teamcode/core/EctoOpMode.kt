package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class EctoOpMode: OpMode() {
    private val controllers: ArrayList<Controller> = arrayListOf()

    protected fun addController(controller: Controller){
        controllers.add(controller)
    }

    final override fun init() {
        telemetry.msTransmissionInterval = 20
        controllers.forEach {
            it.telemetry = telemetry
            it.init(hardwareMap)
        }
    }

    final override fun start() {
        controllers.forEach {
            it.start()
        }
    }

    final override fun stop() {
        controllers.forEach {
            it.stop()
        }
    }
}