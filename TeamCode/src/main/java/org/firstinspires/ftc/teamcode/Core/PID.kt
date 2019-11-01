package org.firstinspires.ftc.teamcode.Core

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.channels.ReceiveChannel
import kotlinx.coroutines.channels.produce

data class PIDSettings(val kP: Double = 0.0, val kI: Double = 0.0, val kD: Double = 0.0)

class PID(val pidSettings: PIDSettings = PIDSettings()) {
    var target: Double = 0.0

    fun CoroutineScope.pidLoop (inputs: ReceiveChannel<Double>): ReceiveChannel<Double> = produce {
        for (currentInput in inputs){
            val error =
            val P =
        }
    }
}