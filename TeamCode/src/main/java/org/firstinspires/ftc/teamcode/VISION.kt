package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.EctoOpMode
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
        chassis.turnToAngle(180.0)
        chassis.runToPosition(Coordinate(0.25, 0.35))
        chassis.turnToAngle(180.0)

        while (isActive) {
            if(vision.isVisible){
                val mult = 1.0.withSign(-vision.lastLocation.y)

                telemetry.addData("mult", mult)

                telemetry.update()

                if(vision.lastLocation.y.absoluteValue < 10) {
                    chassis.movementTarget = MecanumMoveCommand(theta = 180.0)
                    break
                } else {
                    chassis.movementTarget = MecanumMoveCommand(vy = mult * 0.3, theta = 180.0)
                }
            }else{
                chassis.movementTarget = MecanumMoveCommand(vy = 0.3, theta = 180.0)
            }

6
            telemetry.addData("visible", vision.isVisible)
            telemetry.addData("y", vision.lastLocation.y)
        }
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
}