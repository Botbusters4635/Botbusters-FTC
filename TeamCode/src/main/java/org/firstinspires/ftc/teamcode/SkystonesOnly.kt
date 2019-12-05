package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.Arm
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.PositionArm
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.Path
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.withSign


@Autonomous(name = "SkystonesOnly", group = "Red")

class SkystonesOnly : EctoLinearOpMode() {

    val vision = VisionController()
    val chassis = PositionChassis()
    val arm = SynchronizedArm()

    init {
        addController(vision)
        addController(chassis)
        addController(arm)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngleBlocking(180.0)

    }
    override fun runOpMode() {
        arm.moveToPosition(ArmPosition.INTAKE)
        chassis.runToPositionBlocking(Coordinate(0.18, -0.2))
        alignVision()

        chassis.runToPositionBlocking(Coordinate(0.18, chassis.getCurrentCords().y - 0.45 ))
        chassis.runToPositionBlocking(Coordinate(0.6, chassis.getCurrentCords().y))
        chassis.turnToAngleBlocking(90.0)
    }

    fun alignVision(){
        var doneAlign = false

        while(!doneAlign){
            telemetry.addData("visible", vision.isVisible)

            if(!vision.isVisible){
                chassis.movementTarget = MecanumMoveCommand(vx= 0.0, vy = -0.1, theta = 180.0)
            }else{
                val sign = -vision.lastLocation.y.sign
                telemetry.addData("sign", sign)
                telemetry.addData("lastLocation", sign)
                telemetry.addData("y", "%.2f",  vision.lastLocation.y)
                telemetry.addData("x", "%.2f",  vision.lastLocation.x)

                chassis.movementTarget = MecanumMoveCommand(vx= 0.0, vy = 0.1 * sign, theta = 180.0)

                if(vision.lastLocation.y.absoluteValue < 10){
                    doneAlign = true
                }
            }
        }
    }

}