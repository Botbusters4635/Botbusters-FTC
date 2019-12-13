package org.firstinspires.ftc.teamcode.core

import kotlin.math.absoluteValue

data class PIDSettings(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0, val continous: Boolean = true, val lowerBound: Double = 0.0, val upperBound: Double = 0.0, val iClearZone : Double = 0.0)

class PID(var pidSettings: PIDSettings = PIDSettings()) {
    var target: Double = 0.0

    var error: Double = 0.0
    var accumulatedError = 0.0

    var pComponent = 0.0
    var iComponent = 0.0
    var dComponent = 0.0

    var lastError = 0.0
    var output = 0.0

    var maxOutput = Double.NaN
    var deadzone = Double.NaN
    var deltaError = 0.0


    fun clear(){
        lastError = 0.0
        error = 0.0
        accumulatedError = 0.0
    }

    fun update(reference: Double, timeStep: Double) : Double {
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

        deltaError = 0.0
        if(timeStep > 0.0){
            deltaError = (error - lastError) / (timeStep)
        }


        if(error.absoluteValue < pidSettings.iClearZone){
            accumulatedError = 0.0
        }

        pComponent = error * pidSettings.kP
        iComponent = accumulatedError * pidSettings.kI
        dComponent = deltaError * pidSettings.kD

        output = pComponent + iComponent + dComponent

        if(!maxOutput.isNaN()){
            output = output.coerceIn(-maxOutput, maxOutput)
        }


        if(!deadzone.isNaN()){
            if(output.absoluteValue < deadzone){
                output = 0.0
            }
        }

        lastError = error

        return output

    }
}