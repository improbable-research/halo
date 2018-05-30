import io.improbable.keanu.plating.PlateBuilder
import io.improbable.keanu.vertices.dbl.DoubleVertex
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class Mirror (var servoPitchMin, var servoPitchMax, var servoRotationMin, var servoRotationMax,
              var expectedLocation: Vector3D,
              var calibrationObservations: List<Observation>) {

    // Facing installation, Z runs towards one, X runs rightwards, Y runs upwards

    var mirrorOffsetFromPivot = GaussianVertex(100.0, 10.0)

    var pivotPoint = findPivotPoint()

    // Assume linear map from servoInput, x,  to corresponding mirror rotation, y: y = mx + c
    var mPitch = GaussianVertex(1.0, 1.0)
    var cPitch = GaussianVertex(1.0, 1.0)
    var mRotation = GaussianVertex(1.0, 1.0)
    var cRotation = GaussianVertex(1.0, 1.0)


    fun findPivotPoint(): Vector3D {

        var m = GaussianVertex(1.0, 1.0)
        var n = GaussianVertex(1.0, 1.0)
        var o = GaussianVertex(1.0, 1.0)

        var plates = PlateBuilder<Observation>()
                .fromIterator(calibrationObservations.iterator())
                .withFactory( {plate, data ->

                    // Placeholder function for trial
                    var one = m * data.planeX + n * data.planeY + o * data.planeZ
                    one.observe(1.0)
                } )

    }

    fun computeMirrorNormal (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var mirrorPitch = mPitch * servoPitchSignal + cPitch
        var mirrorRotation = mRotation * servoRotationSignal + cRotation
        return Vector3D(mirrorPitch.cos() * mirrorRotation.cos(),
                        mirrorPitch.sin() * mirrorRotation.cos(),
                           mirrorRotation.sin())
    }

    fun computeMirrorCentrePoint (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var mirrorNormal = computeMirrorNormal(servoPitchSignal, servoRotationSignal)
        var vectorFromPivotToMirrorCentre = mirrorNormal * mirrorOffsetFromPivot
        return pivotPoint + vectorFromPivotToMirrorCentre
    }

    fun computeMirrorPlane (servoPitchSignal: DoubleVertex, servoRotationSignal: DoubleVertex): Vector3D {
        var mirrorCentreLocation = computeMirrorCentrePoint(servoPitchSignal, servoRotationSignal)
        var mirrorNormal = computeMirrorNormal(servoPitchSignal, servoRotationSignal)
        return mirrorNormal * mirrorCentreLocation.dot(mirrorNormal)
    }


    fun getMirrorNormal (incomingSunDirection: Vector3D, targetPoint: Vector3D) {

        var pivotToTargetDirection = (targetPoint - pivotPoint).unit()

        var mirrorNormal = ((incomingSunDirection + pivotToTargetDirection) / 2.0).cross(incomingSunDirection.cross(pivotToTargetDirection))

    }

}