package org.firstinspires.ftc.teamcode.controllers

import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.core.Controller

class TimestepTester(val name: String, val delay: Double = 0.0): Controller() {

    override fun update(timeStep: Double) {
        telemetry.addData("timestep of $name ", timeStep * 1000.0)
        runBlocking {
            delay(delay.toLong())
        }
    }
}