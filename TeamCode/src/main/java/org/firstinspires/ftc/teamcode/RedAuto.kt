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

@Autonomous(name = "RedAuto", group = "Red")
class RedAuto : EctoLinearOpMode() {
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
        chassis.moveTimed(MecanumMoveCommand(vy = -0.15, theta = chassis.heading), 4.0)


        arm.moveToPosition(ArmPosition.INTAKE)

        intake.power = 1.0

        chassis.runToPositionBlocking(Coordinate(0.15, chassis.getCurrentCords().y))

        intake.power = 0.0

        arm.moveToPosition(ArmPosition.PASSBRIDGE)

        chassis.turnToAngleBlocking(90.0)

        trayHolder.setPosition(TrayHolderPosition.Release)

        runBlocking {
            delay(500)
        }

        chassis.runToPositionBlocking(Coordinate(0.05, 0.60))

        chassis.turnToAngleBlocking(60.0)

        intake.power = -0.7

        chassis.moveTimed(MecanumMoveCommand(vx = 0.25, theta = chassis.heading), 3.2)
        chassis.runToPositionBlocking(Coordinate(0.05, 0.66))
        chassis.turnToAngleBlocking(-90.0)
        intake.power = 1.0

        chassis.moveTimed(MecanumMoveCommand(vx = 3.0, theta = chassis.heading),  0.5)
        chassis.moveTimed(MecanumMoveCommand(vx = -3.0, theta = chassis.heading), 0.5)
        chassis.runToPositionBlocking(Coordinate(0.05, 0.58))

    }
}


