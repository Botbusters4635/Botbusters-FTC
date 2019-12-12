package org.firstinspires.ftc.teamcode.LinearOpModes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.controllers.Intake
import org.firstinspires.ftc.teamcode.controllers.chassis.MecanumMoveCommand
import org.firstinspires.ftc.teamcode.controllers.chassis.PositionChassis
import org.firstinspires.ftc.teamcode.controllers.arm.ArmPosition
import org.firstinspires.ftc.teamcode.controllers.arm.SynchronizedArm
import org.firstinspires.ftc.teamcode.core.Coordinate
import org.firstinspires.ftc.teamcode.core.EctoLinearOpMode

@Autonomous(name = "OnlyGrabBlue", group = "Blue")
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
        chassis.runToPositionBlocking(Coordinate(0.275, 0.0))
        chassis.turnToAngleBlocking(-45.0)
        intake.power = 1.0
        chassis.moveTimed(MecanumMoveCommand(vx = 0.1, theta = -45.0), 3.5)
        intake.power = 0.0
        chassis.runToPositionBlocking(Coordinate(0.1, 0.0))
        chassis.runToPositionBlocking(Coordinate(0.1, 0.2))
        chassis.turnToAngleBlocking(90.0)
        intake.power = -1.0
        chassis.maxAutoVx = 0.5
        chassis.maxAutoVy = 0.5
        chassis.runToPositionBlocking(Coordinate(0.2, 0.1))
    }

}