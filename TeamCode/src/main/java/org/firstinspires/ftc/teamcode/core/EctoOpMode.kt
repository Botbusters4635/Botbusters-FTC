package org.firstinspires.ftc.teamcode.core

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking

abstract class EctoOpMode : OpMode() {
    private val controllers: ArrayList<Controller> = arrayListOf()
    private val lastTimeRun: ArrayList<Double> = arrayListOf(0.0)

    var initialTimeSet = false
    val updateRate = 10 //Milliseconds

    protected fun addController(controller: Controller) {
        controllers.add(controller)
        lastTimeRun.add(0.0)
    }

    final override fun init() {
        telemetry.msTransmissionInterval = updateRate
        controllers.forEach {
            it.telemetry = telemetry
            it.init(hardwareMap)
        }
    }

    final override fun loop() {
        controllers.withIndex().forEach {
            val timeStep = runtime - lastTimeRun[it.index + 1]
            lastTimeRun[it.index + 1] = runtime
            it.value.update(timeStep)
        }

        val timeStep = runtime - lastTimeRun[0]

        lastTimeRun[0] = runtime

        update(timeStep)

        runBlocking {
            delay( (updateRate.toLong() - timeStep.toLong()).coerceAtLeast(0.0.toLong()))
        }

    }

    abstract fun update(timeStep: Double)

    open fun startMode() {

    }

    final override fun start() {
        lastTimeRun[0] = runtime
        controllers.withIndex().forEach {
            it.value.start()
            lastTimeRun[it.index + 1] = runtime
        }
        startMode()
    }

    final override fun stop() {
        controllers.forEach {
            it.stop()
        }
    }
}