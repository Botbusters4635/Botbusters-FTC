package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.*
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.controllers.chassis.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.chassis.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.clamp.Clamp
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "BlueAuto", group = "Blue")
class BlueAuto : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val arm = SynchronizedArm()
    val intake = Intake()
    val clamp = Clamp()
    val trayHolder = TrayHolder()

    init {
        controllers.add(chassis)
        controllers.add(arm)
        controllers.add(intake)
        controllers.add(clamp)
        controllers.add(trayHolder)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngleBlocking(180.0)

    }

    override fun runOpMode() {

        chassis.maxAutoVx = 0.3
        intake.power = 1.0
        arm.moveToPosition(ArmPosition.HOME)

        chassis.runToPositionBlocking(Coordinate(0.39, 0.1))

        trayHolder.setPosition(TrayHolderPosition.Grab)

        runBlocking {
            delay(500)
        }

        arm.moveToPosition(ArmPosition.INTAKE)

        intake.power = 1.0

        chassis.runToPositionBlocking(Coordinate(0.2, -0.1))

        intake.power = 0.0

        arm.moveToPosition(ArmPosition.PASSBRIDGE)

        chassis.turnToAngleBlocking(-90.0)

        trayHolder.setPosition(TrayHolderPosition.Release)

        runBlocking {
            delay(500)
        }

        chassis.maxAutoVx = 0.3


        chassis.runToPositionBlocking(Coordinate(0.1, -0.58))

        chassis.turnToAngleBlocking(-50.0)

        intake.power = -1.0

        chassis.moveTimed(MecanumMoveCommand(vx = 0.15, theta = chassis.heading), 3.2)

        intake.power = 1.0

        chassis.runToPositionBlocking(Coordinate(0.1, -0.66))
        chassis.turnToAngleBlocking(90.0)

        chassis.moveTimed(MecanumMoveCommand(vx = 2.6, theta = chassis.heading),  0.5)
        chassis.moveTimed(MecanumMoveCommand(vx = 2.6, theta = chassis.heading), 0.5)
        chassis.runToPositionBlocking(Coordinate(0.1, -0.58))

    }
}


