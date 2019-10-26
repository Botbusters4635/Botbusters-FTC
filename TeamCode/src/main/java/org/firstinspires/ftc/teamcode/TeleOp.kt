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
        chassis.move(Twist2D(vx = 0.0, vy = 0.0, w = 0.0))
telemetry.addData("current heading", chassis.getHeading()
)
    }

}