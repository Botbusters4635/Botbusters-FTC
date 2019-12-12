package org.firstinspires.ftc.teamcode

import android.os.SystemClock
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.system.measureTimeMillis

fun measureTimeAndPrint(telemetry: Telemetry, name: String, block: () -> Unit) {
    val timeMeasured = measureTimeMillis {
        block()
    }
    telemetry.addData("Measured time of $name", timeMeasured)
}

class ValueCacher<T>(initialValue: T, val validTimeframeMillis: Double = 20.0) {
    var lastValue = initialValue
    var lastTimeRead = 0.0

    init {
        lastTimeRead = SystemClock.elapsedRealtime().toDouble()
    }

    fun cachedGet(getter: () -> T): () -> T {
        return {
            if (SystemClock.elapsedRealtime() - lastTimeRead > validTimeframeMillis) {
                lastValue = getter()
                lastTimeRead = SystemClock.elapsedRealtime().toDouble()
            }
            lastValue
        }
    }
}