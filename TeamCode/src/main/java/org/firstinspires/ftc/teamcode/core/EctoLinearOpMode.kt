package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.*


abstract class EctoLinearOpMode : EctoOpMode() {
    private var linearOpModeController = LinearOpModeController()

    init {
        addController(linearOpModeController)
    }

    override fun update() {

    }

    abstract fun runOpMode()

    protected inner class LinearOpModeController : Controller() {
        private var scope = CoroutineScope(Job())

        override fun init(hardwareMap: HardwareMap) {

        }

        override fun start() {
            scope.launch {
                runOpMode()
                requestOpModeStop()
            }
        }

        override fun update() {

        }

        override fun stop() {
            scope.coroutineContext.cancelChildren()
        }

    }
}