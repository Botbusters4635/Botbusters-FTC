package org.firstinspires.ftc.teamcode.controllers.arm

import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.core.Coordinate

open class PositionArm : Arm() {

    protected val kinematics = ArmKinematics(armLenght1 = .26, armLength2 = .26)

    var targetCoordinate = ArmPosition.HOGAR.coordinate
        set(value) {
            val targetAngles = kinematics.calculateInverseKinematics(value)
            lowerAngleTarget = targetAngles.lowerAngle
            upperAngleTarget = targetAngles.upperAngle
            field = value
        }

    val currentCoordinate: Coordinate
        get() {
            return kinematics.calculateFowardKinematics(currentAngles)
        }

    open fun moveToPosition(position: ArmPosition) {
        targetCoordinate = position.coordinate
    }

    fun runToPositionBlocking(position: ArmPosition) = runBlocking {
        moveToPosition(position)
        while (!currentCoordinate.closeTo(position.coordinate)) {

        }
    }
}


