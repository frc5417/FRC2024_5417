package frc.robot;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

public class Regression {
    
    public static double[] quadRegression(double[] x, double[] y) {
        final WeightedObservedPoints obs = new WeightedObservedPoints();
        for (int i = 0; i < x.length; i++){
            obs.add(x[i], y[i]);
        }
        final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(2);
        final double[] coeff = fitter.fit(obs.toList());

        return coeff;
    }

}
