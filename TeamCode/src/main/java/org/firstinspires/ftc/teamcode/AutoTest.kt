package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Controllers.Chassis
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.Twist2D

@TeleOp(name = "TeleOp2")
class AutoTest : EctoLinearOpMode() {
    val chassis = Chassis()

    init {
        addController(chassis)
    }

    override fun runOpMode() {
        while (isActive) {
            val targetVelocity = -gamepad1.right_stick_x.toDouble()

            val twist = Twist2D(vx = -gamepad1.left_stick_y.toDouble(), vy = -gamepad1.left_stick_x.toDouble(), w = targetVelocity)

            chassis.movementTarget = twist
        }
    }
}