package org.firstinspires.ftc.teamcode.Systems

import org.firstinspires.ftc.teamcode.core.Controller
class MecanumMotorValues{
    public var topLeftSpeed = 0.0
    public var topRightSpeed = 0.0
    public var downLeftSpeed = 0.0
    public var downRightSpeed = 0.0
}
class MecanumKinematics{
    fun calcInverseKinematics (vx: Double , vy: Double , w: Double){

    }
}
/**
 * moveBy(vx, vy, w)
 *
 */

class Chassis: Controller(){
    fun move (vx: Double , vy: Double , w: Double)
}
