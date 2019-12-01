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
import kotlin.math.withSign


@Autonomous(name = "ads")

class VISION : EctoLinearOpMode() {

//    val vision = VisionController()
//    val chassis = PositionChassis()
//    val arm = SynchronizedArm()

    val trayHolder = TrayHolder()
    init {
//        addController(vision)
//        addController(chassis)
//        addController(arm)
        addController(trayHolder)
    }

    override fun startMode() {

    }
    override fun runOpMode() {

//            trayHolder.setPosition(TrayHolderPosition.Release)
//            telemetry.addData("Pos",TrayHolderPosition.Release )
//            telemetry.update()
//            runBlocking {
//                delay(1000)
//            }
//            trayHolder.setPosition(TrayHolderPosition.Grab)
//            telemetry.addData("Pos",TrayHolderPosition.Grab )
//            telemetry.update()
//            runBlocking {
//                delay(1000)
//            }

//        chassis.heading = 180.0
//        chassis.turnToAngleBlocking(180.0)
//        chassis.runToPositionBlocking(Coordinate(0.5, 0.0))



//        alignWithTarget(SearchDirection.Right)
//
//        arm.runToPositionBlocking(ArmPosition.SECOND_LEVEL)
//        arm.clamp.angle = 90.0
//        runBlocking {
//            delay(1000)
//        }
//        arm.runToPositionBlocking(ArmPosition.SLOW)
//        runBlocking {
//            delay(1000)
//        }
    }

    enum class SearchDirection {
        Right,
        Left
    }

    private fun alignWithTarget(searchDirection: SearchDirection){
//        var aligned = false
//        val velocity = if(searchDirection == SearchDirection.Left) 0.3 else - 0.3
//
//        while (!aligned) {
//            var command: MecanumMoveCommand
//            if(vision.isVisible){
//                val mult = 1.0.withSign(vision.lastLocation.y)
//
//                if(vision.lastLocation.y.absoluteValue < 10) {
//                    command = MecanumMoveCommand(theta = 180.0)
//                    aligned = true
//                } else {
//                    command = MecanumMoveCommand(vy = mult * velocity, theta = 180.0)
//                }
//            }else{
//                command = MecanumMoveCommand(vy = velocity, theta = 180.0)
//            }
//
//            chassis.movementTarget = command
//        }
    }
}