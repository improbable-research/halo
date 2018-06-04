import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import io.improbable.keanu.kotlin.*

// Note: Sign convention:
// Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

class HelioStat (var params: ProbabilisticHelioStatParameters) {

    var heliostatOffsetFromPivot = 0.144 // measured with caliper, variance sub-millimetre

    constructor(params : HelioStatParameters) : this(ProbabilisticHelioStatParameters(params)) {}


    fun servoSignalToUnitSpherical(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex) : ProbabilisticVector3D {
        return ProbabilisticVector3D(ConstantDoubleVertex(1.0),
                                  params.pitchParameters.m * servoPitchSignal + params.pitchParameters.c,
                                  params.rotationParameters.m * servoRotationSignal + params.rotationParameters.c)
    }

    fun servoSignalToUnitSpherical(control : ServoSetting) : ProbabilisticVector3D {
        return servoSignalToUnitSpherical(ConstantDoubleVertex(control.pitch.toDouble()),
                                          ConstantDoubleVertex(control.rotation.toDouble()))
    }

    fun sphericalToCartesian(pitch : DoubleVertex, rotation : DoubleVertex) : ProbabilisticVector3D {
        return ProbabilisticVector3D(pitch.sin() * rotation.cos(),
                                        pitch.cos(),
                                     pitch.sin() * rotation.sin())
    }

    fun computeHeliostatNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val mirrorAngle = servoSignalToUnitSpherical(servoPitchSignal, servoRotationSignal)
        return sphericalToCartesian(mirrorAngle.y, mirrorAngle.z)
    }

    fun computeHeliostatCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        val vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return params.pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        val heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computeHeliostatPlaneSpherical (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val sphericalNorm = servoSignalToUnitSpherical(servoPitchSignal, servoRotationSignal)
        val cartesianNorm = sphericalToCartesian(sphericalNorm.y, sphericalNorm.z)
        return ProbabilisticVector3D(params.pivotPoint.dot(cartesianNorm) + heliostatOffsetFromPivot,
                                        sphericalNorm.y,
                                        sphericalNorm.z)
    }

    fun computePlaneDistanceFromOrigin(pitch : Double, rotation : Double): DoubleVertex {
        val normal = Geometry.sphericalToCartesian(Vector3D(1.0,pitch,rotation))
        return params.pivotPoint.dot(normal) + heliostatOffsetFromPivot
    }

    fun computeTargetFromSourceDirection(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                                         incomingLightDirection: ProbabilisticVector3D,
                                         distance: DoubleVertex): ProbabilisticVector3D {
        val heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        val heliostatPlane = computeHeliostatPlane(servoPitchSignal, servoRotationSignal)
        val unitNormal = heliostatPlane.unit()
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

    fun rotateVectorAroundAxisByTheta(vector: ProbabilisticVector3D, axis: ProbabilisticVector3D, theta: Double): ProbabilisticVector3D {
        val u = axis
        val cosTheta = Math.cos(theta)
        val oneMinusCosTheta = 1.0 - cosTheta
        val sinTheta = Math.sin(theta)

        var resultX = vector.x * (cosTheta + u.x * u.x * oneMinusCosTheta)
        resultX *= u.x * u.y * oneMinusCosTheta - u.z * sinTheta
        resultX *= u.x * u.z * oneMinusCosTheta + u.y * sinTheta

        var resultY = vector.y * (u.y * u.x * oneMinusCosTheta + u.z * sinTheta)
        resultY *= cosTheta + u.y * u.y * oneMinusCosTheta
        resultY *= u.y * u.z * oneMinusCosTheta - u.x * sinTheta

        var resultZ = vector.z * (u.z * u.x * oneMinusCosTheta - u.y * sinTheta)
        resultZ *= u.z * u.y * oneMinusCosTheta + u.x * sinTheta
        resultZ *= cosTheta + u.z * u.z * oneMinusCosTheta

        return ProbabilisticVector3D(resultX, resultY, resultZ)
    }
}
