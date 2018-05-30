import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex

class Observation (rotation: Double, pitch: Double, planeX: Double, planeY: Double, planeZ: Double) {

    var servoError = 1.0
    var measurementError = 1.0

    var rotation = GaussianVertex(rotation, servoError)
    var pitch = GaussianVertex(pitch, servoError)

    var planeX = GaussianVertex(planeX, measurementError)
    var planeY = GaussianVertex(planeY, measurementError)
    var planeZ = GaussianVertex(planeZ, measurementError)

    var planeNormal = Vector3D(this.planeX, this.planeY, this.planeZ)
    var controlSetting = Pair(this.rotation, this.pitch)

}