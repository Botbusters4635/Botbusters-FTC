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

@Autonomous(name = "BlueAuto", group = "Blue")
class BlueAuto : EctoLinearOpMode() {
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
        intake.power = 1.0
        arm.moveToPosition(ArmPosition.HOME)

        chassis.runToPositionBlocking(Coordinate(0.39, 0.1))

        trayHolder.setPosition(TrayHolderPosition.Grab)

        runBlocking {
            delay(500)
        }

        arm.moveToPosition(ArmPosition.INTAKE)

        intake.power = 1.0

//        chassis.maxAutoVx = 0.1
        chassis.runToPositionBlocking(Coordinate(0.1, -0.1))

        intake.power = 0.0
//        chassis.turnToAngleBlocking(20.0)
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

        chassis.moveTimed(MecanumMoveCommand(vx = 0.25, theta = chassis.heading), 3.2)

        intake.power = 1.0

        chassis.runToPositionBlocking(Coordinate(0.1, -0.66))
//        intake.power = 1.0
        chassis.turnToAngleBlocking(90.0)


        //leAFFAE
        chassis.moveTimed(MecanumMoveCommand(vx = 2.6, theta = chassis.heading),  0.5)
        chassis.moveTimed(MecanumMoveCommand(vx = 2.6, theta = chassis.heading), 0.5)
        chassis.runToPositionBlocking(Coordinate(0.1, -0.58))

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


