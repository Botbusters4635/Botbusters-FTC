package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.Chassis
import org.firstinspires.ftc.teamcode.controllers.TimestepTester
import org.firstinspires.ftc.teamcode.core.EctoOpMode

@Autonomous(name="timestep")
class TimestepMode: EctoOpMode() {
    val chassis = Chassis()
    var targetHeading = 0.0
    val maxTargetHeadingRate = 45
    var rotations = 0.0
    init {
        addController(TimestepTester("first", 10.0))
    }
    override fun update(timeStep: Double) {

        val desiredChange = -gamepad1.right_stick_x.toDouble()
        targetHeading += (desiredChange * maxTargetHeadingRate) * timeStep

        if (targetHeading > 180.0) {
            targetHeading -= 360
            rotations++
        }

        if (targetHeading < -180.0) {
            targetHeading += 360
            rotations--
        }

        telemetry.addData("currentHeading" , targetHeading)
        telemetry.addData("rotations" , rotations)
        telemetry.addData("opMode timestep", timeStep * 1000)

    }
}