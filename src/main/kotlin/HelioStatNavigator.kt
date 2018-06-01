import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D
import kotlin.math.roundToInt

class HelioStatNavigator(var params: ProbabilisticHelioStatParameters) {

    fun computeServoSetting(incomingSunDirection: ProbabilisticVector3D,
                            targetPoint: Vector3D): ServoSetting {

        val model = HelioStat(params)

        // Todo update prior to match actual servo range
        val servoPitchRange = UniformVertex(0.0, 4096.0)
        val servoRotationRange = UniformVertex(0.0, 4096.0)
        val targetDistance = UniformVertex(1.0, 20.0)

        val target = model.computeTargetPoint(servoPitchRange, servoRotationRange, incomingSunDirection, targetDistance)

        val targetObservationNoise = Vector3D(1.0, 1.0, 1.0)
        target.noisyObserve(targetPoint, targetObservationNoise)

        val net = BayesNet(target.x.connectedGraph)

        val gradOpt = GradientOptimizer(net)
        gradOpt.maxAPosteriori(1000)

        return ServoSetting(servoRotationRange.value.roundToInt(), servoPitchRange.value.roundToInt())
    }
}