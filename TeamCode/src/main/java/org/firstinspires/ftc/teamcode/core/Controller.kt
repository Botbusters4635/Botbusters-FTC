package org.firstinspires.ftc.teamcode.core

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.*
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class Controller {
    private val scope = CoroutineScope(Job())

    lateinit var telemetry: Telemetry

    lateinit var controllers: ControllerManager

    fun periodicFunction(delay: Double = 0.0, block: () -> Unit): () -> Unit {
        return {
            scope.launch {
                while (scope.isActive) {
                    block()
                    delay(delay.toLong())
                }
            }

        }
    }

    open fun init(hardwareMap: HardwareMap) {}

    open fun start() {}

    open fun update(timeStep: Double) {}

    fun stop() {
        scope.coroutineContext.cancelChildren()
    }
}
