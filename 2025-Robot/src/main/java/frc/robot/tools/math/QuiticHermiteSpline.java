package frc.robot.tools.math;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

public class QuiticHermiteSpline {
    final Matrix<N6, N6> scale = new Matrix<N6, N6>(new SimpleMatrix(6, 6, true, new float[] {
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, (float) 0.5, 0, 0, 0,
            -10, -6, (float) -1.5, 10, -4, (float) 0.5,
            15, 8, (float) 1.5, -15, 7, -1,
            -6, -3, (float) -0.5, 6, -3, (float) 0.5
    }));
    final int numSegments;
    final Matrix<N6, N1>[] segmentCoefficients;
    final Derivatives[] derivatives;

    /**
     * Constructs a quintic Hermite spline from the given derivatives.
     *
     * @param derivatives An array of Derivatives objects representing the
     *                    derivatives at each waypoint.
     */
    @SuppressWarnings("unchecked")
    public QuiticHermiteSpline(Derivatives[] derivatives) {
        this.derivatives = derivatives;
        numSegments = derivatives.length - 1;
        segmentCoefficients = new Matrix[numSegments];
        for (int i = 0; i < numSegments; i++) {
            segmentCoefficients[i] = new Matrix<N6, N1>(new SimpleMatrix(6, 1, true, new float[] {
                    (float) derivatives[i].d0,
                    (float) derivatives[i].d1,
                    (float) derivatives[i].d2,
                    (float) derivatives[i + 1].d0,
                    (float) derivatives[i + 1].d1,
                    (float) derivatives[i + 1].d2
            }));
            segmentCoefficients[i] = scale.times(segmentCoefficients[i]);
        }
    }

    /**
     * Interpolates the derivatives at a given time using the quintic Hermite
     * spline.
     *
     * @param time The time at which to interpolate the derivatives.
     * @return The derivatives at the specified time.
     * @throws IndexOutOfBoundsException if the time is out of bounds for the spline
     *                                   segments.
     */
    Derivatives interpolate(double time) {
        // Check if the time is within the bounds of the spline segments
        if (time < derivatives[0].t || time > derivatives[numSegments].t) {
            throw new IndexOutOfBoundsException("Time out of bounds for spline segments");
        }

        // Find the segment index for the given time
        int segmentIndex = 0;
        for (int i = 0; i < numSegments; i++) {
            if (time < derivatives[i + 1].t) {
                segmentIndex = i;
                break;
            }
        }

        // Calculate the time relative to the start of the segment
        double t = time - derivatives[segmentIndex].t;
        Matrix<N6, N1> coefficients = segmentCoefficients[segmentIndex];

        // Calculate the powers of t
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        // Calculate the derivatives at the given time
        double d0 = coefficients.get(0, 0) + coefficients.get(1, 0) * t + coefficients.get(2, 0) * t2
                + coefficients.get(3, 0) * t3 + coefficients.get(4, 0) * t4 + coefficients.get(5, 0) * t5;
        double d1 = coefficients.get(1, 0) + 2 * coefficients.get(2, 0) * t + 3 * coefficients.get(3, 0) * t2
                + 4 * coefficients.get(4, 0) * t3 + 5 * coefficients.get(5, 0) * t4;
        double d2 = 2 * coefficients.get(2, 0) + 6 * coefficients.get(3, 0) * t
                + 12 * coefficients.get(4, 0) * t2 + 20 * coefficients.get(5, 0) * t3;

        return new Derivatives(time, d0, d1, d2);
    }
}