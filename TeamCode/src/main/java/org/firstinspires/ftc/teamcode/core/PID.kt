package org.firstinspires.ftc.teamcode.core

import android.os.SystemClock
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.*

data class PIDSettings(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0, val continous: Boolean = true, val lowerBound: Double = 0.0, val upperBound: Double = 0.0)

class PID(var pidSettings: PIDSettings = PIDSettings()) {
    var target: Double = 0.0

    var error: Double = 0.0
    var accumulatedError = 0.0

    var pComponent = 0.0
    var iComponent = 0.0
    var dComponent = 0.0

    var lastTimeUpdate =  SystemClock.elapsedRealtime() / 1000.0
    var lastError = 0.0
    var output = 0.0

    fun update(reference: Double) : Double {

        val timeStep = SystemClock.elapsedRealtime() / 1000.0 - lastTimeUpdate

        error = target - reference // Current input received from the producer
        if (pidSettings.continous) {
            if (error > pidSettings.upperBound) {
                error -= pidSettings.upperBound - pidSettings.lowerBound
            }
            if (error < pidSettings.lowerBound) {
                error += pidSettings.upperBound - pidSettings.lowerBound
            }
        }

        accumulatedError += error * (timeStep)

        val deltaError = (error - lastError) / (timeStep)


        pComponent = error * pidSettings.kP
        iComponent = accumulatedError * pidSettings.kI
        dComponent = deltaError * pidSettings.kD

        output = pComponent + iComponent + dComponent

        lastError = error
        lastTimeUpdate =  SystemClock.elapsedRealtime() / 1000.0

        return output

    }
}