import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class Query {

    class Navigation(var params: HelioStatParameters, var currentServoSetting: ServoSetting,
                     var targetPoint: Vector3D, var source: Vector3D)

    class SetNormal(var params: HelioStatParameters, var mirrorNormal: Vector3D)

}
