package org.firstinspires.ftc.teamcode.LinearOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.Intake
import org.firstinspires.ftc.teamcode.controllers.chassis.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.chassis.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "OnlyGrabRed", group = "Red")
class OnlyGrabRed : EctoLinearOpMode() {

    val chassis = PositionChassis()
    val arm = SynchronizedArm()
    val intake = Intake()

    init {
        controllers.add(chassis)
        controllers.add(arm)
        controllers.add(intake)
    }

    override fun startMode() {
        arm.moveToPosition(ArmPosition.PASSBRIDGE)
    }

    override fun runOpMode() {
        chassis.runToPositionBlocking(Coordinate(0.5, -0.15))
        chassis.turnToAngleBlocking(40.0)
        intake.power = -1.0
        chassis.moveTimed(MecanumMoveCommand(vx = 0.15, theta = 40.0), 1.8)
        runBlocking {
            delay(1000)
        }
        intake.power = 0.0
        chassis.runToPositionBlocking(Coordinate(0.1, -0.2))
        chassis.maxAutoVx = 0.7
        chassis.maxAutoVy = 0.7
        chassis.turnToAngleBlocking(-90.0)
        runBlocking { delay(500) }
        intake.power = 1.0
        chassis.moveTimed(MecanumMoveCommand(vx = 0.7, theta = -90.0), 1.0)
        chassis.moveTimed(MecanumMoveCommand(vx = -0.7, theta = -90.0), 0.8)


        chassis.runToPositionBlocking(Coordinate(0.7, -0.35))

        runBlocking { delay(10000) }
    }

}