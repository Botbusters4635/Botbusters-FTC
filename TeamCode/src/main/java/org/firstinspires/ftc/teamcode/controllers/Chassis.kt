package org.firstinspires.ftc.teamcode.controllers

import android.os.SystemClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.core.*
import kotlin.math.cos
import kotlin.math.sin


data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0) {
    operator fun plus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed + other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed + other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed + other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed + other.downRightSpeed
        return result
    }

    operator fun minus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed - other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed - other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed - other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed - other.downRightSpeed
        return result
    }
}

data class MecanumMoveCommand(var vx: Double = 0.0, var vy: Double = 0.0, var theta: Double = 0.0)

class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.topRightSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downLeftSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downRightSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        return values

    }


    fun calcForwardKinematics(state: MecanumMotorValues): Twist2D {
        val result = Twist2D()

        result.vx = (state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0
        result.vy = (-state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed - state.downRightSpeed) * wheelRadius / 4.0
        result.w = (-state.topLeftSpeed + state.topRightSpeed - state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0 * (xDistanceFromWheelToCenter + yDistanceFromWheelToCenter)
        return result
    }
}


open class Chassis : Controller() {
    private val pidSettingsNormal = PIDSettings(kP = 0.15, kI = 0.00, kD = 0.0025, continous = true, lowerBound = -180.0, upperBound = 180.0)

    private val angularPID = PID(pidSettingsNormal)

    private lateinit var topLeftMotor: DcMotorEx
    private lateinit var topRightMotor: DcMotorEx
    private lateinit var downRightMotor: DcMotorEx
    private lateinit var downLeftMotor: DcMotorEx

    private lateinit var imu: BNO055IMU

    private var kinematics = MecanumKinematics(0.25/2, 0.27/2, 0.1016)

    protected var currentCoords = Coordinate()

    private var lastTimeRun = SystemClock.elapsedRealtime() / 1000.0
    var timeStep = 0.0

    private var offset = 0.0

    var heading: Double
        set(value) {
            offset = value - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
        }
        get() {
            val currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble() - offset

            if (currentHeading > 180) return currentHeading - 360
            else if (currentHeading < -180) return currentHeading + 360
            else return currentHeading
        }

    var movementTarget = MecanumMoveCommand()


    override fun init(hardwareMap: HardwareMap) {
        // Get motors and cast to DcMotorEx
        topLeftMotor = hardwareMap.get(DcMotor::class.java, "topLeftMotor") as DcMotorEx
        topRightMotor = hardwareMap.get(DcMotor::class.java, "topRightMotor") as DcMotorEx
        downLeftMotor = hardwareMap.get(DcMotor::class.java, "downLeftMotor") as DcMotorEx
        downRightMotor = hardwareMap.get(DcMotor::class.java, "downRightMotor") as DcMotorEx


        topLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        topRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        downLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reverse motors
        topLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        downLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        // Get the IMU, configure it and initialize it.
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.NDOF
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imu.initialize(parameters)

        // Initialize the angularPID

    }


    override fun update() {

        timeStep = SystemClock.elapsedRealtime() / 1000.0 - lastTimeRun

        updateCurrentCoords(timeStep)
        angularPID.target = movementTarget.theta

        val motorValues: MecanumMotorValues = kinematics.calcInverseKinematics(
                movementTarget.vx,
                movementTarget.vy,
                angularPID.update(heading)
        )

        writeMotors(motorValues)
        lastTimeRun = SystemClock.elapsedRealtime() / 1000.0

        telemetry.addData("position", currentCoords)
        telemetry.addData("topLeftPos", topLeftMotor.currentPosition)
        telemetry.addData("topRightPos", topRightMotor.currentPosition)
        telemetry.addData("downLeftPos", downLeftMotor.currentPosition)
        telemetry.addData("downRightPos", downRightMotor.currentPosition)

    }

    fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.setVelocity(values.topLeftSpeed, AngleUnit.RADIANS)
        topRightMotor.setVelocity(values.topRightSpeed, AngleUnit.RADIANS)
        downLeftMotor.setVelocity(values.downLeftSpeed, AngleUnit.RADIANS)
        downRightMotor.setVelocity(values.downRightSpeed, AngleUnit.RADIANS)
    }

    fun getLocalVelocities(): Twist2D {
        val wheelsSpeed = MecanumMotorValues()

        wheelsSpeed.topLeftSpeed = topLeftMotor.getVelocity(AngleUnit.RADIANS)

        wheelsSpeed.topRightSpeed = topRightMotor.getVelocity(AngleUnit.RADIANS)

        wheelsSpeed.downLeftSpeed = downLeftMotor.getVelocity(AngleUnit.RADIANS)

        wheelsSpeed.downRightSpeed = downRightMotor.getVelocity(AngleUnit.RADIANS)

        val localVelocities = kinematics.calcForwardKinematics(wheelsSpeed)

        return localVelocities
    }

    fun degreesToRadians(degrees: Double): Double {
        return degrees * Math.PI / 180.0
    }

    fun getGlobalVelocities(): Twist2D {
        val localVelocities = getLocalVelocities()

        val headinginRadians = degreesToRadians(heading)

        val globalVelocities = Twist2D()

        /**
         * Removed Vy because tires slip, may give cleaner output for autonomous period
         */
        globalVelocities.vx = localVelocities.vx * cos(headinginRadians) - localVelocities.vy * sin(headinginRadians)
        globalVelocities.vy = localVelocities.vy * cos(headinginRadians) + localVelocities.vx * sin(headinginRadians)

        return globalVelocities

    }

    fun updateCurrentCoords(timeStep: Double) {
        val globalVelocities = getGlobalVelocities()

        currentCoords.x = currentCoords.x + (globalVelocities.vx * timeStep * 0.143)
        currentCoords.y = currentCoords.y + (globalVelocities.vy * timeStep * 0.143)
    }

    fun getCurrentCords(): Coordinate {
        return currentCoords
    }
}
