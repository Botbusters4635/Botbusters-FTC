package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.chassis.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.chassis.PositionChassis
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "Chassis presentation", group = "presentation")
class PresentChassis : EctoLinearOpMode() {
    val chassis = PositionChassis()

    init {
        controllers.add(chassis)
    }

    override fun runOpMode() {
        chassis.moveTimed(MecanumMoveCommand(vy = -0.1), 1.0)
        runBlocking {
            delay(500)
        }
        chassis.moveTimed(MecanumMoveCommand(vy = 0.1), 1.0)
        runBlocking {
            delay(500)
        }
        chassis.moveTimed(MecanumMoveCommand(vx = 0.1), 1.0)
        runBlocking {
            delay(500)
        }
        chassis.moveTimed(MecanumMoveCommand(vx = -0.1), 1.0)
    }

}