package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Core.EctoOpMode
import org.firstinspires.ftc.teamcode.systems.Chassis
import org.firstinspires.ftc.teamcode.systems.Twist2D

@TeleOp(name = "TeleOp")
class TeleOp: EctoOpMode() {
    val chassis = Chassis()
    init {
        addController(chassis)
    }

    override fun loop() {
        chassis.move(Twist2D(vx = gamepad1.left_stick_y.toDouble(), vy = gamepad1.left_stick_x.toDouble(), w = gamepad1.right_stick_x.toDouble()))
    }

}