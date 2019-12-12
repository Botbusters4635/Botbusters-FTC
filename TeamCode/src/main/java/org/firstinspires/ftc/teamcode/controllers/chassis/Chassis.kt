package org.firstinspires.ftc.teamcode.controllers.chassis

import android.os.SystemClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.controllers.motors.EctoDcMotor
import org.firstinspires.ftc.teamcode.controllers.sensors.EctoBNO055IMU
import org.firstinspires.ftc.teamcode.core.*
import org.firstinspires.ftc.teamcode.measureTimeAndPrint
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin


open class Chassis : Controller() {
    private val pidSettingsNormal = PIDSettings(kP = 0.075, kI = 0.00, kD = 0.005, continous = true, lowerBound = -180.0, upperBound = 180.0)

    protected val angularPID = PID(pidSettingsNormal)

    var angularSpeedTarget: Double = 0.0
        set(value) {
            field = value
            pidActive = false
        }

    var pidActive = true

    private var topLeftMotor = EctoDcMotor("topLeftMotor")
    private var topRightMotor = EctoDcMotor("topRightMotor")
    private var downRightMotor = EctoDcMotor("downRightMotor")
    private var downLeftMotor = EctoDcMotor("downLeftMotor")

    private var imu = EctoBNO055IMU("imu")

    private var kinematics = MecanumKinematics(0.25 / 2, 0.27 / 2, 0.0508)

    protected var currentCoords = Coordinate()
    private var offset = 0.0

    var heading: Double
        set(value) {
            offset = value - imu.heading
        }
        get() {

            val currentHeading = imu.heading - offset

            return when {
                currentHeading > 180 -> currentHeading - 360
                currentHeading < -180 -> currentHeading + 360
                else -> currentHeading
            }

        }

    val maxV = 0.6

    var movementTarget: MecanumMoveCommand = MecanumMoveCommand()
        set(value) {
            if (value.vx.absoluteValue > maxV) {
                value.vx = Math.copySign(maxV, value.vx)
            }

            if (value.vy.absoluteValue > maxV) {
                value.vy = Math.copySign(maxV, value.vy)
            }

            field = value
        }


    override fun init(hardwareMap: HardwareMap) {
        // Get motors and cast to DcMotorEx

        controllers.add(downLeftMotor)
        controllers.add(downRightMotor)
        controllers.add(topLeftMotor)
        controllers.add(topRightMotor)
        controllers.add(imu)



        topLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        topRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reverse motors
        topLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        downLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        // Get the IMU, configure it and initialize it.


//        val filename = "AdafruitIMUCalibration.json"
//        val file = AppUtil.getInstance().getSettingsFile(filename)
//
//        imu.writeCalibrationData(BNO055IMU.CalibrationData.deserialize(file.readText()))

        angularPID.deadzone = 0.1
    }


    override fun update(timeStep: Double) {

        updateCurrentCoords(timeStep)

        val motorValues: MecanumMotorValues

        if (pidActive) {
            angularPID.target = movementTarget.theta
            motorValues = kinematics.calcInverseKinematics(
                    movementTarget.vx,
                    movementTarget.vy,
                    angularPID.update(heading, timeStep)
            )
        } else {
            motorValues = kinematics.calcInverseKinematics(
                    movementTarget.vx,
                    movementTarget.vy,
                    angularSpeedTarget
            )
        }
        writeMotors(motorValues)
    }

    fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.velocity = values.topLeftSpeed
        topRightMotor.velocity = values.topRightSpeed
        downLeftMotor.velocity = values.downLeftSpeed
        downRightMotor.velocity = values.downRightSpeed

    }

    fun getLocalVelocities(): Twist2D {

        val wheelsSpeed = MecanumMotorValues()

        wheelsSpeed.topLeftSpeed = topLeftMotor.velocity * 1.25

        wheelsSpeed.topRightSpeed = topRightMotor.velocity * 1.25

        wheelsSpeed.downLeftSpeed = downLeftMotor.velocity * 1.25

        wheelsSpeed.downRightSpeed = downRightMotor.velocity * 1.25

        return kinematics.calcForwardKinematics(wheelsSpeed)
    }

    fun degreesToRadians(degrees: Double): Double {
        return degrees * Math.PI / 180.0
    }

    fun getGlobalVelocities(): Twist2D {
        val localVelocities = getLocalVelocities()

        val headinginRadians = degreesToRadians(heading)

        val globalVelocities = Twist2D()

        globalVelocities.vx = localVelocities.vx * cos(headinginRadians) - localVelocities.vy * sin(headinginRadians)
        globalVelocities.vy = localVelocities.vy * cos(headinginRadians) + localVelocities.vx * sin(headinginRadians)

        return globalVelocities

    }

    fun updateCurrentCoords(timeStep: Double) {
        val globalVelocities = getGlobalVelocities()

        currentCoords.x = currentCoords.x + (globalVelocities.vx * timeStep) * 8.097165992
        currentCoords.y = currentCoords.y + (globalVelocities.vy * timeStep) * 8.097165992
    }

    fun getCurrentCords(): Coordinate {
        return currentCoords
    }
}
