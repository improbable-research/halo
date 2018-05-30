import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D

class HelioStatNavigator(var params: ProbabilisticHelioStatParameters) {


    fun computeServoSetting(incomingSunDirection: ProbabilisticVector3D,
                            targetPoint: Vector3D): ServoSetting {

        var model = HelioStat(params)

        var servoPitchRange = UniformVertex(-100.0, 100.0)
        var servoRotationRange = UniformVertex(-100.0, 100.0)
        var targetDistance = UniformVertex(1.0, 20.0)

        var target = model.computeTargetPoint(servoPitchRange, servoRotationRange, incomingSunDirection, targetDistance)

        var targetObservationNoise = Vector3D(1.0, 1.0, 1.0)
        target.noisyObserve(targetPoint, targetObservationNoise)

        var net = BayesNet(target.x.connectedGraph)

        var gradOpt = GradientOptimizer(net)
        gradOpt.maxAPosteriori(1000)

        return ServoSetting(servoRotationRange.value.toInt(), servoPitchRange.value.toInt())
    }
}