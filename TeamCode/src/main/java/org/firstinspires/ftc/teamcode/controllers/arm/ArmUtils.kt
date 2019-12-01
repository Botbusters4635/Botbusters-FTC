package org.firstinspires.ftc.teamcode.controllers.arm

import org.firstinspires.ftc.teamcode.core.Coordinate

data class ArmAngleValues(var lowerAngle: Double = 0.00, var upperAngle: Double = 0.00)

enum class ArmState {
    GO_TARGET, EXCHANGE_BACK_TO_FRONT, EXCHANGE_FRONT_TO_BACK
}

enum class TURN_POS(val value: Double) {
    OPEN(0.0), CLOSED(1.0), MIDDLE(0.5)
}

enum class ArmPosition(val coordinate: Coordinate) {
    SLOW(Coordinate(-0.42, 0.2)),
    HOME(Coordinate(0.20, 0.12)),
    FIRST_LEVEL(Coordinate(-0.26, 0.05)),
    SECOND_LEVEL(Coordinate(-0.26, 0.16)),
    THIRD_LEVEL(Coordinate(-0.26, 0.3)),
    FOURTH_LEVEL(Coordinate(-0.26, 0.37)),
    FIFTH_LEVEL(Coordinate(-0.26, 0.40)),
    EXCHANGE(Coordinate(0.25, 0.32)),
    INTAKE(Coordinate(0.25, 0.28)),
    PASSBRIDGE(Coordinate(0.47, 0.17))
}