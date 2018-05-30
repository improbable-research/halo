import io.improbable.keanu.plating.PlateBuilder
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class Mirror (var expectedLocation: Vector3D,
              var calibrationObservations: List<Observation>) {

    var mirrorOffsetFromPivot = GaussianVertex(100.0, 10.0)

    var pivotPoint = findPivotPoint()


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


    fun getMirrorNormal (incomingSunDirection: Vector3D, targetPoint: Vector3D) {

        var pivotToTargetDirection = (targetPoint - pivotPoint).unit()

        var mirrorNormal = ((incomingSunDirection + pivotToTargetDirection) / 2.0).cross(incomingSunDirection.cross(pivotToTargetDirection))

    }

}