package org.firstinspires.ftc.teamcode.Core

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class EctoOpMode: OpMode() {
    private val controllers: ArrayList<Controller> = arrayListOf()

    protected fun addController(controller: Controller){
        controllers.add(controller)
    }

    final override fun init() {
        controllers.forEach {
            it.init(telemetry, hardwareMap)
        }
    }
}