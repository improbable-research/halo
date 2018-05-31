import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.nonprobabilistic.ConstantDoubleVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

// Note: Sign convention:
// Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

class HelioStat (var params: ProbabilisticHelioStatParameters) {

    var heliostatOffsetFromPivot = 0.144 // measured with caliper, variance sub-millimetre

    fun servoSignalToUnitSpherical(servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex) : ProbabilisticVector3D {
        return ProbabilisticVector3D(
                ConstantDoubleVertex(1.0),
                params.pitchParameters.m * servoPitchSignal + params.pitchParameters.c,
                params.rotationParameters.m * servoRotationSignal + params.rotationParameters.c
        )
    }

    fun servoSignalToUnitSpherical(control : ServoSetting) : ProbabilisticVector3D {
        return servoSignalToUnitSpherical(
                ConstantDoubleVertex(control.pitch.toDouble()),
                ConstantDoubleVertex(control.rotation.toDouble())
        )
    }

    fun sphericalToCartesian(pitch : DoubleVertex, rotation : DoubleVertex) : ProbabilisticVector3D {
        return ProbabilisticVector3D(
                pitch.sin() * rotation.cos(),
                pitch.cos(),
                pitch.sin() * rotation.sin()
        )
    }

    fun computeHeliostatNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val mirrorAngle = servoSignalToUnitSpherical(servoPitchSignal, servoRotationSignal)
        return sphericalToCartesian(mirrorAngle.y, mirrorAngle.z)
    }

    fun computeHeliostatCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        var vectorFromPivotToHeliostatCentre = heliostatNormal * heliostatOffsetFromPivot
        return params.pivotPoint + vectorFromPivotToHeliostatCentre
    }

    fun computeHeliostatPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatNormal = computeHeliostatNormal(servoPitchSignal, servoRotationSignal)
        return heliostatNormal * heliostatCentreLocation.dot(heliostatNormal)
    }

    fun computeHeliostatPlaneSpherical (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): ProbabilisticVector3D {
        val sphericalNorm = servoSignalToUnitSpherical(servoPitchSignal, servoRotationSignal)
        val cartesianNorm = sphericalToCartesian(sphericalNorm.y, sphericalNorm.z)
        return ProbabilisticVector3D(
                params.pivotPoint.dot(cartesianNorm) + heliostatOffsetFromPivot,
                sphericalNorm.y,
                sphericalNorm.z
        )
    }


//    fun computePlaneNorm (normal: ProbabilisticVector3D): DoubleVertex {
//        return params.pivotPoint.dot(normal) + heliostatOffsetFromPivot
//    }

    fun computePlaneDistanceFromOrigin(pitch : Double, rotation : Double): DoubleVertex {
        val normal = Geometry.sphericalToCartesian(Vector3D(1.0,pitch,rotation))
        return params.pivotPoint.dot(normal) + heliostatOffsetFromPivot
    }


    fun computeTargetPoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex,
                            incomingSunDirection: ProbabilisticVector3D, distance: DoubleVertex): ProbabilisticVector3D {
        var heliostatCentreLocation = computeHeliostatCentrePoint(servoPitchSignal, servoRotationSignal)
        var heliostatPlane = computeHeliostatPlane(servoPitchSignal, servoRotationSignal)
        var unitNormal = heliostatPlane.unit()
        var reflectedDirection = incomingSunDirection - unitNormal * incomingSunDirection.dot(unitNormal) * 2.0
        return heliostatCentreLocation + reflectedDirection * distance
    }


}