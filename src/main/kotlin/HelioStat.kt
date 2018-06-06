import io.improbable.keanu.kotlin.cos
import io.improbable.keanu.kotlin.minus
import io.improbable.keanu.kotlin.sin
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

// Note: Sign convention:
// Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

class HelioStat(var params: ProbabilisticHelioStatParameters) {

    //    var heliostatOffsetFromPivot = 0.144 // measured with caliper, variance sub-millimetre
    var heliostatOffsetFromPivot = 0.145 // measured with caliper, variance sub-millimetre

    constructor(params: HelioStatParameters) : this(ProbabilisticHelioStatParameters(params)) {}

    fun getLogLikelihood(dataPoint: CalibrationData.DataPoint): Double {
        val normSigma2 = 0.05 * 0.05
        val lengthSigma2 = 0.05 * 0.05
        val normal = computeHeliostatNormal(dataPoint.control).getValue()
        val length = computePlaneDistanceFromOrigin(dataPoint.pitch, dataPoint.rotation).value
        val observedNormal = Geometry.sphericalToCartesian(Vector3D(1.0, dataPoint.pitch, dataPoint.rotation))
        return (-normal.subtract(observedNormal).normSq / (2.0 * normSigma2) + 0.5 * Math.log(2.0 * Math.PI * normSigma2) -
                Math.pow(length - dataPoint.length, 2.0) / (2.0 * lengthSigma2) + 0.5 * Math.log(2.0 * Math.PI * lengthSigma2))
    }

    fun computeHeliostatNormal(control: ServoSetting): ProbabilisticVector3D {
        return computeHeliostatNormal(ConstantDoubleVertex(control.pitch.toDouble()),
                ConstantDoubleVertex(control.rotation.toDouble()))
    }

    fun linearSphericalNormalModel(control: ServoSetting): ProbabilisticVector3D {
        val servoPitch = params.pitchParameters.m * control.pitch.toDouble() + params.pitchParameters.c
        val servoRotation = params.rotationParameters.m * control.rotation.toDouble() + params.rotationParameters.c
        return ProbabilisticVector3D(ConstantDoubleVertex(1.0), servoPitch, servoRotation)
    }


    fun computeHeliostatNormal(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val servoPitch = params.pitchParameters.m * servoPitchSignal + params.pitchParameters.c
        val servoRotation = params.rotationParameters.m * servoRotationSignal + params.rotationParameters.c
        val pitchAxis = ProbabilisticVector3D(ConstantDoubleVertex(1.0), params.pitchParameters.axisPitch, params.pitchParameters.axisRotation)
        val cartesianPitchAxis = pitchAxis.sphericalToCartesian()
        val rotationAxis = ProbabilisticVector3D(ConstantDoubleVertex(1.0), params.rotationParameters.axisPitch, params.rotationParameters.axisRotation)
        val cartesianRotationAxis = rotationAxis.sphericalToCartesian()
        var result = cartesianRotationAxis * (-1.0)
        result = rotateVectorAroundAxisByTheta(result, cartesianPitchAxis, servoPitch)
        result = rotateVectorAroundAxisByTheta(result, cartesianRotationAxis, servoRotation)
        return result
    }

    fun computeHeliostatCentrePoint(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        val vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return params.pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        val heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computePlaneDistanceFromOrigin(pitch: Double, rotation: Double): DoubleVertex {
        val normal = Geometry.sphericalToCartesian(Vector3D(1.0, pitch, rotation))
        return params.pivotPoint.dot(normal) + heliostatOffsetFromPivot
    }


    fun computeTargetFromSourceDirection(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                                         incomingLightDirection: ProbabilisticVector3D,
                                         distance: DoubleVertex): ProbabilisticVector3D {
        val heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        val unitNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        val reflectedDirection = incomingLightDirection - unitNormal * unitNormal.dot(incomingLightDirection) * 2.0
        return heliostatCentreLocation + reflectedDirection * distance
    }

    fun computeTargetFromSourcePoint(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                                     sourcePoint: Vector3D,
                                     distance: DoubleVertex): ProbabilisticVector3D {
        val centrePoint = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        val incomingLightDirection = ProbabilisticVector3D(sourcePoint) - centrePoint
        return computeTargetFromSourceDirection(servoPitchSignal, servoRotationSignal, incomingLightDirection, distance)
    }

    fun rotateVectorAroundAxisByTheta(vector: ProbabilisticVector3D, axis: ProbabilisticVector3D, theta: DoubleVertex): ProbabilisticVector3D {
        val u = axis
        val cosTheta = cos(theta)
        val oneMinusCosTheta = 1.0 - cosTheta
        val sinTheta = sin(theta)

        var resultX = vector.x * (cosTheta + u.x * u.x * oneMinusCosTheta) +
                vector.y * (u.x * u.y * oneMinusCosTheta - u.z * sinTheta) +
                vector.z * (u.x * u.z * oneMinusCosTheta + u.y * sinTheta)

        var resultY = vector.x * (u.y * u.x * oneMinusCosTheta + u.z * sinTheta) +
                vector.y * (cosTheta + u.y * u.y * oneMinusCosTheta) +
                vector.z * (u.y * u.z * oneMinusCosTheta - u.x * sinTheta)

        var resultZ = vector.x * (u.z * u.x * oneMinusCosTheta - u.y * sinTheta) +
                vector.y * (u.z * u.y * oneMinusCosTheta + u.x * sinTheta) +
                vector.z * (cosTheta + u.z * u.z * oneMinusCosTheta)

        return ProbabilisticVector3D(resultX, resultY, resultZ)
    }

//     --- OLD SPHERICAL STUFF
//
//    fun computeHeliostatNormalSpherical(control: ServoSetting): ProbabilisticVector3D {
//        return computeHeliostatNormal(ConstantDoubleVertex(control.pitch.toDouble()),
//                ConstantDoubleVertex(control.rotation.toDouble())).cartesianToSpherical()
//    }

//    fun computeHeliostatPlaneSpherical(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
//        val cartesianNorm = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
//        val sphericalNorm = cartesianNorm.cartesianToSpherical()
//        return ProbabilisticVector3D(params.pivotPoint.dot(cartesianNorm) + heliostatOffsetFromPivot,
//                sphericalNorm.y,
//                sphericalNorm.z)
//    }


}
