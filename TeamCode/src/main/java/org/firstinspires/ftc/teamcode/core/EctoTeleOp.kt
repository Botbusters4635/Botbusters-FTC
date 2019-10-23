package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.Controller

abstract class EctoTeleOp: OpMode() {
    val controllers: ArrayList<Controller> = arrayListOf()

    fun addController(newController: Controller){
        controllers.add(newController)
    }
    override fun init() {
        controllers.forEach{
            it.init()
            it.telemetry = telemetry
            it.hardwareMap = hardwareMap
        }
    }
    override fun loop() {
        controllers.forEach {
            it.update()
        }
    }
}