package org.firstinspires.ftc.teamcode.core

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking

abstract class EctoOpMode: OpMode() {
    private val controllers: ArrayList<Controller> = arrayListOf()

    private var lastTimeRun = 0.0
    var initialTimeSet = false
    val updateRate = 10 //Milliseconds

    protected fun addController(controller: Controller){
        controllers.add(controller)
    }

    final override fun init() {
        telemetry.msTransmissionInterval = updateRate
        controllers.forEach {
            it.telemetry = telemetry
            it.init(hardwareMap)
        }
    }

    final override fun loop() {
        if(!initialTimeSet){
            lastTimeRun = runtime
            initialTimeSet = true
            return
        }
        val timeStep = runtime - lastTimeRun
        controllers.forEach{
            it.update(timeStep)
        }
        update(timeStep)

        lastTimeRun = runtime
        runBlocking {
            delay(updateRate.toLong())
        }

    }

    abstract fun update(timeStep: Double)

    open fun startMode(){

    }

    final override fun start() {
        controllers.forEach {
            it.start()
        }
        startMode()
    }

    final override fun stop() {
        controllers.forEach {
            it.stop()
        }
    }
}