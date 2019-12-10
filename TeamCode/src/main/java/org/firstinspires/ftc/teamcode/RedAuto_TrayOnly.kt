package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "RedAuto_TrayOnly", group = "Red")
class RedAuto_TrayOnly : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val arm = SynchronizedArm()
    val intake = Intake()
    val clamp = Clamp()
    val trayHolder = TrayHolder()

    init {
        addController(chassis)
        addController(arm)
        addController(intake)
        addController(clamp)
        addController(trayHolder)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngleBlocking(180.0)

    }

    override fun runOpMode() {
        chassis.maxAutoVx = 0.3
        chassis.maxAutoVy = 0.3
        chassis.maxAutoAngular = 0.9

        intake.rightPower = 1.0
        arm.moveToPosition(ArmPosition.HOME)

        chassis.runToPositionBlocking(Coordinate(0.4, -0.1))

        trayHolder.setPosition(TrayHolderPosition.Grab)
        runBlocking {
            delay(800)
        }
        chassis.moveTimed(MecanumMoveCommand(vy = -0.15, theta = chassis.heading), 3.35)


        arm.moveToPosition(ArmPosition.INTAKE)

        intake.power = 1.0

        chassis.runToPositionBlocking(Coordinate(0.15, chassis.getCurrentCords().y))

        intake.power = 0.0

        arm.moveToPosition(ArmPosition.PASSBRIDGE)

        chassis.turnToAngleBlocking(90.0)

        trayHolder.setPosition(TrayHolderPosition.Release)

//        chassis.moveTimed(MecanumMoveCommand(vx = -2.0, theta = 90.0), 0.75)

        runBlocking {
            delay(500)
        }

        chassis.runToPositionBlocking(Coordinate(0.25, 0.60))
    }
}


