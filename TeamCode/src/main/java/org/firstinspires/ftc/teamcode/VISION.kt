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
        chassis.turnToAngle(180.0)
        chassis.runToPosition(Coordinate(0.3, 0.35))
        chassis.turnToAngle(180.0)

        while (isActive) {
            var mult = 1.0
            if(vision.isVisible){
                runBlocking {
                    delay(200)
                }
                mult = Math.copySign(1.0, vision.lastLocation.y)

                if(vision.lastLocation.y.absoluteValue < 10) break
            }
            telemetry.addData("mult", mult)
            telemetry.addData("visible", vision.isVisible)
            telemetry.addData("x", vision.lastLocation.x)
            telemetry.update()

            chassis.movementTarget = MecanumMoveCommand(vy = mult * 0.3, theta = 180.0)


        }
        chassis.movementTarget = MecanumMoveCommand(theta = 180.0)
    }
}