package org.firstinspires.ftc.teamcode.controllers.arm

import org.firstinspires.ftc.teamcode.core.Coordinate
import kotlin.math.*

class ArmKinematics(val armLenght1: Double, val armLength2: Double) {


    fun calculateInverseKinematics(targetCoord: Coordinate): ArmAngleValues {
        val values = ArmAngleValues()
        var cosAngle = (targetCoord.x.pow(2) + targetCoord.y.pow(2) - armLenght1.pow(2) - armLength2.pow(2)) / (2 * armLenght1 * armLength2)

        if (cosAngle > 1) {
            cosAngle = 1.0
        } else if (cosAngle < -1.0) {
            cosAngle = -1.0
        }


        if (targetCoord.x > 0) {
            values.upperAngle = -acos(cosAngle)
            values.lowerAngle = atan2(targetCoord.y, targetCoord.x) + atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        } else {
            values.upperAngle = acos(cosAngle)
            values.lowerAngle = atan2(targetCoord.y, targetCoord.x) - atan2((armLength2 * sin(abs(values.upperAngle))), (armLenght1 + armLength2 * cos(abs(values.upperAngle))))
        }

        values.upperAngle *= 180.0 / Math.PI
        values.lowerAngle *= 180.0 / Math.PI

        return values

    }

    fun calculateFowardKinematics(armValues: ArmAngleValues): Coordinate {
        val values2 = Coordinate()
        armValues.lowerAngle *= Math.PI / 180.0
        armValues.upperAngle *= Math.PI / 180.0

        values2.x = cos(armValues.lowerAngle) * armLenght1 + cos(armValues.lowerAngle + armValues.upperAngle) * armLength2
        values2.y = sin(armValues.lowerAngle) * armLenght1 + sin(armValues.lowerAngle + armValues.upperAngle) * armLength2

        return values2
    }


}