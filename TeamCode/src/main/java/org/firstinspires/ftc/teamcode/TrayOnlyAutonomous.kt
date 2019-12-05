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

@Autonomous(name = "TrayOnly", group = "Red")
class TrayOnlyAutonomous : EctoLinearOpMode() {
    val chassis = PositionChassis()
    val arm = SynchronizedArm()

    init {
        addController(chassis)
        addController(arm)
    }

    override fun startMode() {
        arm.moveToPosition(ArmPosition.PASSBRIDGE)
    }

    override fun runOpMode() {

        chassis.runToPositionBlocking(Coordinate(0.3, 0.0))

        chassis.maxAutoVx = 0.1

        chassis.runToPositionBlocking(Coordinate(0.44, 0.0))

        arm.moveToPosition(ArmPosition.HOME)

        chassis.turnToAngleBlocking(5.0)

        chassis.turnToAngleBlocking(-5.0)

        chassis.turnToAngleBlocking(0.0)

        runBlocking {
            delay(1000)
        }

//        chassis.maxAutoVx = 0.1

        chassis.runToPositionBlocking(Coordinate(-0.05, 0.0))

//        chassis.turnToAngleBlocking(20.0)

        chassis.turnToAngleBlocking(-90.0)

        chassis.maxAutoVx = 0.6

        chassis.runToPositionBlocking(Coordinate(-0.05, 0.2))

        arm.runToPositionBlocking(ArmPosition.PASSBRIDGE)

        chassis.runToPositionBlocking(Coordinate(-0.05, 0.6))

        chassis.turnToAngleBlocking(45.0)

    }

//        chassis.runToPositionBlocking(Coordinate(0.75, 0.0))

//        chassis.runToPositionBlocking(Coordinate(0.1, -0.5))
    /***
     * Enable grabbing cosa
     */
//        trayHolder.setPosition(TrayHolderPosition.Grab)
//        runBlocking {
//            delay(250)
//        }
//
//        /**q
//
//        chassis.runToPositionBlocking(Coordinate(0.5, 0.0))
//
//
//        /**
//         * Girar para acomodarla y empujarla contra la zona
//         */
//        chassis.turnToAngleBlocking(90.0)
//        arm.moveToPosition(ArmPosition.PASSBRIDGE)
//        chassis.runToPositionBlocking(Coordinate(chassis.getCurrentCords().x, chassis.getCurrentCords().y - 0.25))
//
//        trayHolder.setPosition(TrayHolderPosition.Release)
//        runBlocking {
//            delay(250)
//        }
//
//        /**
//         * Ir a estacionarse
//         */
//        chassis.runToPositionBlocking(Coordinate(chassis.getCurrentCords().x, 1.0))
}


