package org.firstinspires.ftc.teamcode

import android.os.SystemClock
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.core.EctoOpMode
import org.firstinspires.ftc.teamcode.systems.Chassis
import org.firstinspires.ftc.teamcode.systems.Twist2D
import kotlin.math.*

@TeleOp(name = "TeleOp")
class TeleOp : EctoOpMode() {
    val chassis = Chassis()
    var targetHeading = 0.0
    val maxTargetHeadingRate = 180
    var lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

    init {
        addController(chassis)
    }

    override fun loop() {
        val timeStep = (SystemClock.elapsedRealtime() / 1000.0) - lastTimeRun

        val targetHeadingDelta = -gamepad1.right_stick_x * maxTargetHeadingRate

        targetHeading += targetHeadingDelta * timeStep


        if (targetHeading > 180) {
            targetHeading -= 360
        }
        if (targetHeading < -180) {
            targetHeading += 360
        }

//        val twist = Twist2D(vx = -gamepad1.left_stick_y.toDouble(), vy = -gamepad1.left_stick_x.toDouble(), w = targetHeading)
        chassis.moveWithoutPID(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble(), targetHeading)
//        chassis.movementTarget = twist
        lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

        SystemClock.sleep(20)
    }

}