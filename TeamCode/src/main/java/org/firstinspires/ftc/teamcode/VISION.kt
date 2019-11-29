package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.Chassis
import org.firstinspires.ftc.teamcode.controllers.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.VisionController
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.EctoOpMode
import kotlin.math.absoluteValue


@TeleOp(name = "ads")

class VISION : EctoLinearOpMode() {

    val vision = VisionController()
    val chassis = PositionChassis()

    init {
        addController(vision)
        addController(chassis)
    }


    override fun runOpMode() {
        chassis.heading = 180.0

        chassis.runToPosition(Coordinate(0.45, 0.35), true)
        chassis.turnToAngle(180.0)

        while (isActive && !vision.isVisible ) {
            chassis.movementTarget = MecanumMoveCommand(vy = 0.3, theta = 180.0)
        }
        while(vision.lastLocation.x.absoluteValue > 30){
            chassis.movementTarget = MecanumMoveCommand(vy = 0.3, theta = 180.0)
        }

        runBlocking {
            delay(250)
        }
        chassis.movementTarget = MecanumMoveCommand(theta = 180.0)
    }
}