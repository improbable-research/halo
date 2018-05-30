import io.improbable.keanu.algorithms.variational.GradientOptimizer
import io.improbable.keanu.network.BayesNet
import io.improbable.keanu.vertices.dbl.probabilistic.GaussianVertex
import io.improbable.keanu.vertices.dbl.probabilistic.UniformVertex

class HelioStatNavigator(var params: ProbabilisticHelioStatParameters) {


    fun computeServoSetting(incomingSunDirection: ProbabilisticVector3D,
                            targetPoint: ProbabilisticVector3D): ServoSetting {

        var model = HelioStat(params)

        var servoPitchRange = UniformVertex(-100.0, 100.0)
        var servoRotationRange = UniformVertex(-100.0, 100.0)
        var targetDistance = UniformVertex(1.0, 20.0)

        var target = model.computeTargetPoint(servoPitchRange, servoRotationRange, incomingSunDirection, targetDistance)

        var targetObservationNoise = ProbabilisticVector3D(GaussianVertex(1.0, 0.0),
                                                           GaussianVertex(1.0, 0.0),
                                                           GaussianVertex(1.0, 0.0))
        target.noisyObserve(targetPoint, targetObservationNoise)

        var net = BayesNet(target.x.connectedGraph)

        var gradOpt = GradientOptimizer(net)
        gradOpt.maxAPosteriori(1000)



    }
}