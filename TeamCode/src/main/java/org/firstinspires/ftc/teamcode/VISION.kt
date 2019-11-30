package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.Arm
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import kotlin.math.absoluteValue
import kotlin.math.withSign


@TeleOp(name = "ads")

class VISION : EctoLinearOpMode() {

    val vision = VisionController()
    val chassis = PositionChassis()
    val arm = Arm()

    init {
        addController(vision)
        addController(chassis)
        addController(arm)
    }


    override fun runOpMode() {
        chassis.heading = 180.0
        chassis.runToPosition(Coordinate(0.25, 0.35))

        alignWithTarget(SearchDirection.Right)

        arm.runToPositionCommand(ArmPosition.MEDIUM)
        arm.setServoHeading(90.0)
        runBlocking {
            delay(1000)
        }
        arm.runToPositionCommand(ArmPosition.SLOW)
        runBlocking {
            delay(1000)
        }
    }

    enum class SearchDirection {
        Right,
        Left
    }

    private fun alignWithTarget(searchDirection: SearchDirection){
        var aligned = false
        val velocity = if(searchDirection == SearchDirection.Left) 0.3 else - 0.3

        while (!aligned) {
            var command: MecanumMoveCommand
            if(vision.isVisible){
                val mult = 1.0.withSign(vision.lastLocation.y)

                if(vision.lastLocation.y.absoluteValue < 10) {
                    command = MecanumMoveCommand(theta = 180.0)
                    aligned = true
                } else {
                    command = MecanumMoveCommand(vy = mult * velocity, theta = 180.0)
                }
            }else{
                command = MecanumMoveCommand(vy = velocity, theta = 180.0)
            }

            chassis.movementTarget = command
        }
    }
}