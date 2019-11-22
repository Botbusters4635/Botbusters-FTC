package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.*

internal class LinearOpModeController(val opMode: () -> Unit) : Controller() {
    val scope = CoroutineScope(Job())

    override fun init(hardwareMap: HardwareMap) {

    }

    override fun start() {
        scope.launch{
            opMode()
        }
    }
    override  fun stop() {
        scope.coroutineContext.cancelChildren()
    }
}

abstract class EctoLinearOpMode : EctoOpMode() {

    private var linearOpModeController: LinearOpModeController


    val isActive: Boolean
    get() {
        this.linearOpModeController.scope
    }

    init {
        linearOpModeController = LinearOpModeController(::runOpMode)
        addController(linearOpModeController)
    }

    override fun loop() {


    }

    abstract fun runOpMode()
}