package org.firstinspires.ftc.teamcode.core

import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*

data class PIDSettings(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0, val timestep: Double = 10.0, val continous: Boolean = true, val lowerBound: Double = 0.0, val upperBound: Double = 0.0)

class PID(var pidSettings: PIDSettings = PIDSettings()){
    var target: Double = 0.0
    val inputChannel = Channel<Double>(Channel.CONFLATED)
    val outputChannel = Channel<Double>(Channel.CONFLATED)

    var error: Double = 0.0
    var accumulatedError = 0.0

    var pComponent = 0.0
    var iComponent = 0.0
    var dComponent = 0.0

    @ExperimentalCoroutinesApi
    fun start() = runBlocking {
        var lastError = 0.0

        while (isActive) {
            error = target - inputChannel.receive() // Current input received from the producer
            if(pidSettings.continous){
                if (error > pidSettings.upperBound) {
                    error -= pidSettings.upperBound - pidSettings.lowerBound
                }
                if (error < pidSettings.lowerBound) {
                    error += pidSettings.upperBound - pidSettings.lowerBound
                }
            }

            accumulatedError += error * (pidSettings.timestep / 1000)

            val deltaError = (error - lastError) / (pidSettings.timestep / 1000)


            pComponent = error * pidSettings.kP
            iComponent = accumulatedError * pidSettings.kI
            dComponent = deltaError * pidSettings.kD

            val correction = pComponent + iComponent + dComponent

            outputChannel.send(correction)
            lastError = error
            delay(timeMillis = pidSettings.timestep.toLong())
        }


    }
}