package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.controllers.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.TrayHolder
import org.firstinspires.ftc.teamcode.controllers.TrayHolderPosition
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode
import org.firstinspires.ftc.teamcode.core.Path

@Autonomous(name = "TrayOnly", group = "ezpz")
class TrayOnlyAutonomous : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val trayHolder = TrayHolder()
    val arm = SynchronizedArm()

    init {
        addController(chassis)
        addController(trayHolder)
        addController(arm)
    }

    override fun startMode() {
        chassis.heading = 180.0
        chassis.turnToAngle(180.0)
        arm.moveToPosition(ArmPosition.INTAKE)
    }

    override fun runOpMode() {
        /**
         * Asume que lo de los servos estará por atras
         */
        chassis.runToPosition(Coordinate(0.7, 0.0))
        /***
         * Enable grabbing cosa
         */
        trayHolder.setPosition(TrayHolderPosition.Grab)
        runBlocking {
            delay(250)
        }

        /**
         * Mover charola más cerca a la zona
         */

        chassis.runToPosition(Coordinate(0.5, 0.0))


        /**
         * Girar para acomodarla y empujarla contra la zona
         */
        chassis.turnToAngle(90.0)
        chassis.runToPosition(Coordinate(chassis.getCurrentCords().x, chassis.getCurrentCords().y - 0.25))

        trayHolder.setPosition(TrayHolderPosition.Release)
        runBlocking {
            delay(250)
        }
        arm.runToPositionCommand(ArmPosition.PASSBRIDGE)

        /**
         * Ir a estacionarse
         */
        chassis.runToPosition(Coordinate(chassis.getCurrentCords().x, 1.0))
    }


}