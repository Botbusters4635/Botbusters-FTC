package org.firstinspires.ftc.teamcode.core

import kotlinx.coroutines.*
import kotlinx.coroutines.NonCancellable.isActive
import kotlinx.coroutines.channels.*
import kotlinx.coroutines.channels.produce
import kotlinx.coroutines.flow.consumeAsFlow
import kotlin.coroutines.CoroutineContext

data class PIDSettings(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0, val timestep: Double = 10.0, val continous: Boolean = true, val lowerBound: Double = 0.0, val upperBound: Double = 0.0)

class PID(val pidSettings: PIDSettings = PIDSettings()){
    var target: Double = 0.0
    val inputChannel = Channel<Double>(Channel.CONFLATED)
    val outputChannel = Channel<Double>(Channel.CONFLATED)

    var error: Double = 0.0

    @ExperimentalCoroutinesApi
    fun start() = runBlocking {
        var accumulatedError = 0.0
        var lastError = 0.0

        while (isActive) {
            error = target - inputChannel.receive() // Current input received from the producer
            if(!pidSettings.continous){
                if (error > pidSettings.upperBound) {
                    error -= pidSettings.upperBound - pidSettings.lowerBound
                }
                if (error < pidSettings.lowerBound) {
                    error += pidSettings.upperBound - pidSettings.lowerBound
                }
            }

            accumulatedError += error * (pidSettings.timestep / 1000)

            val deltaError = (error - lastError) / (pidSettings.timestep / 1000)


            val p = error * pidSettings.kP
            val i = accumulatedError * pidSettings.kI
            val d = deltaError * pidSettings.kD

            val correction = p + i + d

            outputChannel.send(correction)

            delay(timeMillis = pidSettings.timestep.toLong())

            lastError = error
        }


    }
}