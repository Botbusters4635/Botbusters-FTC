package org.firstinspires.ftc.teamcode.core

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking

abstract class EctoOpMode : OpMode() {

    var lastTimeRun = 0.0

    val updateRate = 10 //Milliseconds

    val controllers = ControllerManager()

    open fun init_impl() {}

    final override fun init() {
        init_impl()
        telemetry.msTransmissionInterval = updateRate
        controllers.telemetry = telemetry
        controllers.hardwareMap = hardwareMap
        controllers.init()
    }

    final override fun loop() {
        controllers.update()

        val timeStep = runtime - lastTimeRun

        lastTimeRun = runtime

        update(timeStep)

        runBlocking {
            delay( (updateRate.toLong() - timeStep.toLong()).coerceAtLeast(0.0.toLong()))
        }

    }

    abstract fun update(timeStep: Double)

    open fun startMode() {

    }

    final override fun start() {
        lastTimeRun = runtime
        controllers.start()
        startMode()
    }

    final override fun stop() {
        controllers.stop()
    }
}