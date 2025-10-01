import com.mvohm.quadruple.Quadruple;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.util.Locale;
import java.util.Vector;

/**
 * Simple mathematical operations interface to allow for different numeric
 * types.
 * Currently implemented for Float, Double, and Quadruple.
 */
interface SimpleMath<T> {
    T add(T a, T b);

    T subtract(T a, T b);

    T multiply(T a, T b);

    T divide(T a, T b);

    T sqrt(T a);

    int compareTo(T a, T b);

    boolean isNaN(T a);

    boolean isInfinite(T a);

    T negate(T a);

    T abs(T a);

    T acos(double a);

    T fromDouble(double a);

    boolean isZeroApprox(T a);
}

class FloatMath implements SimpleMath<Float> {
    public Float add(Float a, Float b) {
        return a + b;
    }

    public Float subtract(Float a, Float b) {
        return a - b;
    }

    public Float multiply(Float a, Float b) {
        return a * b;
    }

    public Float divide(Float a, Float b) {
        return a / b;
    }

    public Float sqrt(Float a) {
        return (float) Math.sqrt(a);
    }

    public int compareTo(Float a, Float b) {
        return a.compareTo(b);
    }

    public boolean isNaN(Float a) {
        return a.isNaN();
    }

    public boolean isInfinite(Float a) {
        return a.isInfinite();
    }

    public Float negate(Float a) {
        return -a;
    }

    public Float abs(Float a) {
        return Math.abs(a);
    }

    public Float acos(double a) {
        return (float) Math.acos(a);
    }

    @Override
    public Float fromDouble(double a) {
        return (float) a;
    }

    public boolean isZeroApprox(Float a) {
        return Math.abs(a) < 1e-6;
    }
}

class DoubleMath implements SimpleMath<Double> {
    public Double add(Double a, Double b) {
        return a + b;
    }

    public Double subtract(Double a, Double b) {
        return a - b;
    }

    public Double multiply(Double a, Double b) {
        return a * b;
    }

    public Double divide(Double a, Double b) {
        return a / b;
    }

    public Double sqrt(Double a) {
        return Math.sqrt(a);
    }

    public int compareTo(Double a, Double b) {
        return a.compareTo(b);
    }

    public boolean isNaN(Double a) {
        return a.isNaN();
    }

    public boolean isInfinite(Double a) {
        return a.isInfinite();
    }

    public Double negate(Double a) {
        return -a;
    }

    public Double abs(Double a) {
        return Math.abs(a);
    }

    public Double acos(double a) {
        return Math.acos(a);
    }

    @Override
    public Double fromDouble(double a) {
        return a;
    }

    public boolean isZeroApprox(Double a) {
        return Math.abs(a) < 1e-12;
    }
}

class QuadrupleMath implements SimpleMath<Quadruple> {
    public Quadruple add(Quadruple a, Quadruple b) {
        return Quadruple.add(a, b);
    }

    public Quadruple subtract(Quadruple a, Quadruple b) {
        return Quadruple.subtract(a, b);
    }

    public Quadruple multiply(Quadruple a, Quadruple b) {
        return Quadruple.multiply(a, b);
    }

    public Quadruple divide(Quadruple a, Quadruple b) {
        return Quadruple.divide(a, b);
    }

    public Quadruple sqrt(Quadruple a) {
        return Quadruple.sqrt(a);
    }

    public int compareTo(Quadruple a, Quadruple b) {
        return a.compareTo(b);
    }

    public boolean isNaN(Quadruple a) {
        return a.isNaN();
    }

    public boolean isInfinite(Quadruple a) {
        return a.isInfinite();
    }

    public Quadruple negate(Quadruple a) {
        return a.negate();
    }

    public Quadruple abs(Quadruple a) {
        return a.abs();
    }

    public Quadruple acos(double a) {
        return new Quadruple(Math.acos(a));
    }

    public Quadruple fromDouble(double a) {
        return new Quadruple(a);
    }

    public boolean isZeroApprox(Quadruple a) {
        return a.abs().compareTo(new Quadruple(1e-30)) < 0;
    }
}

/**
 * Vector3 class supporting generic numeric types.
 * 
 */
class Vector3<T extends Number> {
    public T x;
    public T y;
    public T z;
    public static final FloatMath FLOAT_MATH = new FloatMath();
    public static final DoubleMath DOUBLE_MATH = new DoubleMath();
    public static final QuadrupleMath QUADRUPLE_MATH = new QuadrupleMath();
    public final SimpleMath<T> math;

    public Vector3(double x, double y, double z, SimpleMath<T> math) {
        this.x = math.fromDouble(x);
        this.y = math.fromDouble(y);
        this.z = math.fromDouble(z);
        this.math = math;
    }

    public Vector3(T x, T y, T z, SimpleMath<T> math) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.math = math;
    }

    public Vector3<T> zero() {
        return new Vector3<T>(math.fromDouble(0), math.fromDouble(0), math.fromDouble(0), math);
    }

    public Vector3<T> up() {
        return new Vector3<T>(math.fromDouble(0.0), math.fromDouble(1.0), math.fromDouble(0.0), math);
    }

    public Vector3<T> normalized() {
        T norm = math.sqrt(math.add(math.add(math.multiply(x, x), math.multiply(y, y)), math.multiply(z, z)));
        return new Vector3<T>(math.divide(x, norm), math.divide(y, norm), math.divide(z, norm), math);
    }

    public Vector3<T> plus(Vector3<T> v) {
        return new Vector3<T>(math.add(x, v.x), math.add(y, v.y), math.add(z, v.z), math);
    }

    public Vector3<T> add(T v) {
        return new Vector3<T>(math.add(x, v), math.add(y, v), math.add(z, v), math);
    }

    public Vector3<T> minus(Vector3<T> v) {
        return new Vector3<T>(math.subtract(x, v.x), math.subtract(y, v.y), math.subtract(z, v.z), math);
    }

    public Vector3<T> subtract(T v) {
        return new Vector3<T>(math.subtract(x, v), math.subtract(y, v), math.subtract(z, v), math);
    }

    public Vector3<T> div(T v) {
        return new Vector3<T>(math.divide(x, v), math.divide(y, v), math.divide(z, v), math);
    }

    public Vector3<T> mul(T v) {
        return new Vector3<T>(math.multiply(x, v), math.multiply(y, v), math.multiply(z, v), math);
    }

    public T dot(Vector3<T> v) {
        return math.add(math.add(math.multiply(x, v.x), math.multiply(y, v.y)), math.multiply(z, v.z));
    }

    public Vector3<T> cross(Vector3<T> v) {
        return new Vector3<T>(
                math.subtract(math.multiply(y, v.z), math.multiply(z, v.y)),
                math.subtract(math.multiply(z, v.x), math.multiply(x, v.z)),
                math.subtract(math.multiply(x, v.y), math.multiply(y, v.x)), math);
    }

    public T distanceTo(Vector3<T> v) {
        T dx = math.subtract(x, v.x);
        T dy = math.subtract(y, v.y);
        T dz = math.subtract(z, v.z);
        return math.sqrt(math.add(math.add(math.multiply(dx, dx), math.multiply(dy, dy)), math.multiply(dz, dz)));
    }

    public T lengthSquared() {
        return math.add(math.add(math.multiply(x, x), math.multiply(y, y)), math.multiply(z, z));
    }

    public T length() {
        return math.sqrt(lengthSquared());
    }

    public boolean isFinite() {
        return !(math.isNaN(x) || math.isInfinite(x) || math.isNaN(y) || math.isInfinite(y) || math.isNaN(z)
                || math.isInfinite(z));
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Vector3))
            return false;
        Vector3<?> v = (Vector3<?>) obj;
        return x.equals(v.x) && y.equals(v.y) && z.equals(v.z);
    }

    @Override
    public String toString() {
        return String.format("Vector3(%.3e, %.3e, %.3e)", x.doubleValue(), y.doubleValue(), z.doubleValue());
    }
}

class Circle<T extends Number> {
    public Vector3<T> center;
    public T radius;
    public final SimpleMath<T> math;

    public Circle(Vector3<T> center, T radius) {
        this.center = center;
        this.radius = radius;
        this.math = center.math;
    }

    public Circle<T> unitCircle() {
        return new Circle<T>(center.zero(), math.fromDouble(1.0));
    }

    public Vector3<T> reflect(Vector3<T> p) {
        if (!center.equals(p)) {
            Vector3<T> diff = p.minus(center);
            return diff.mul(math.divide(math.multiply(radius, radius), diff.lengthSquared())).plus(center);
        } else {
            return p;
        }
    }

    public static <T extends Number> Circle<T> threePointCircle(Vector3<T> p1, Vector3<T> p2, Vector3<T> p3) {
        SimpleMath<T> math = p1.math;
        // Calculate determinant
        T det = math.multiply(math.fromDouble(2.0),
                math.add(
                        math.add(
                                math.multiply(p1.x, math.subtract(p2.z, p3.z)),
                                math.multiply(p2.x, math.subtract(p3.z, p1.z))),
                        math.multiply(p3.x, math.subtract(p1.z, p2.z))));

        // Check if points are collinear
        if (det.equals(math.fromDouble(0.0))) {
            return null;
        }

        // Calculate center x coordinate
        T ux = math.divide(
                math.add(
                        math.add(
                                math.multiply(math.add(math.multiply(p1.x, p1.x), math.multiply(p1.z, p1.z)),
                                        math.subtract(p2.z, p3.z)),
                                math.multiply(math.add(math.multiply(p2.x, p2.x), math.multiply(p2.z, p2.z)),
                                        math.subtract(p3.z, p1.z))),
                        math.multiply(math.add(math.multiply(p3.x, p3.x), math.multiply(p3.z, p3.z)),
                                math.subtract(p1.z, p2.z))),
                det);

        // Calculate center z coordinate
        T uz = math.divide(
                math.add(
                        math.add(
                                math.multiply(math.add(math.multiply(p1.x, p1.x), math.multiply(p1.z, p1.z)),
                                        math.subtract(p3.x, p2.x)),
                                math.multiply(math.add(math.multiply(p2.x, p2.x), math.multiply(p2.z, p2.z)),
                                        math.subtract(p1.x, p3.x))),
                        math.multiply(math.add(math.multiply(p3.x, p3.x), math.multiply(p3.z, p3.z)),
                                math.subtract(p2.x, p1.x))),
                det);

        Vector3<T> center = new Vector3<T>(ux, math.fromDouble(0.0), uz, math);
        T radius = center.distanceTo(p1);
        return new Circle<T>(center, radius);
    }

    @Override
    public String toString() {
        return String.format("Circle(center=(%.6f, %.6f, %.6f), radius=%.6f)",
                center.x.doubleValue(), center.y.doubleValue(), center.z.doubleValue(), radius.doubleValue());
    }
}

class HyperbolicSegment<T extends Number> extends Circle<T> {
    public boolean isColinear;
    public Vector3<T> direction;
    public double startAngle;
    public double sweepAngle;
    public Vector3<T> ab;

    public HyperbolicSegment(Vector3<T> p1, Vector3<T> p2) {
        super(p1.zero(), p1.math.fromDouble(0.0)); // Temporary initialization

        Vector3<T> A = p1;
        Vector3<T> B = p2;

        ab = B.minus(A);
        boolean colinear = GeometricAnalysis.isColinear(A, B);

        this.isColinear = colinear;

        if (colinear) {
            this.direction = ab.normalized();
            this.radius = A.math.fromDouble(Double.POSITIVE_INFINITY);
            this.center = A;
        } else {
            Circle<T> circle = GeometricAnalysis.hyperbolicLine(A, B);

            this.center = circle.center;
            this.radius = circle.radius;

            Vector3<T> xa = A.minus(circle.center);
            Vector3<T> xb = B.minus(circle.center);

            double angleA = Math.atan2(-xa.z.doubleValue(), xa.x.doubleValue());
            double angleB = Math.atan2(-xb.z.doubleValue(), xb.x.doubleValue());
            this.startAngle = angleA;

            double diffAngle = angleB - angleA;

            if (diffAngle > Math.PI) {
                diffAngle = 2.0 * Math.PI - diffAngle;
            } else if (diffAngle < -Math.PI) {
                diffAngle = -(diffAngle + 2.0 * Math.PI);
            }

            double modA = (angleA + diffAngle + 2.0 * Math.PI) % (2.0 * Math.PI);
            double modB = (angleB + 2.0 * Math.PI) % (2.0 * Math.PI);

            if (Vector3.DOUBLE_MATH.isZeroApprox(Math.abs(modA - modB))) {
                this.sweepAngle = diffAngle;
            } else {
                this.sweepAngle = -diffAngle;
            }
        }
    }

    @Override
    public Vector3<T> reflect(Vector3<T> p) {
        Vector3<T> q;
        if (isColinear) {
            // Calculate the vector from line_start to the point
            Vector3<T> pointVec = p.minus(center);

            // Project the point vector onto the line direction
            T projectionLength = pointVec.dot(direction);
            Vector3<T> projection = direction.mul(projectionLength);

            // Calculate the distance from the point to the line
            Vector3<T> distance = pointVec.minus(projection);

            // Reflect the point across the line
            q = p.minus(distance.mul(math.fromDouble(2.0)));
        } else if (math.isNaN(radius)) {
            q = p;
        } else {
            q = super.reflect(p);
        }
        return q;
    }

    public Vector3<T> getPoint(double t) {
        Vector3<T> p;
        if (isColinear) {
            p = center.plus(ab.mul(math.fromDouble(t)));
        } else {
            double angle = startAngle + t * sweepAngle;
            p = center.plus(new Vector3<T>(Math.cos(angle), 0, -Math.sin(angle), math).mul(radius));
        }
        return p;
    }
}

class IntersectionResult<T extends Number> {
    public Vector3<T> intersection1;
    public Vector3<T> intersection2;

    public IntersectionResult(Vector3<T> i1, Vector3<T> i2) {
        this.intersection1 = i1;
        this.intersection2 = i2;
    }
}

class TransformHB<T extends Number> {
    private Circle<T> circle0;
    private Circle<T> circle1;

    TransformHB(Vector3<T> from, Vector3<T> to, HyperbolicCircleFactory<T> factory) {
        Vector3<T> half = to.plus(from).mul(from.math.fromDouble(0.5));
        circle0 = GeometricAnalysis.moveCircle(from, half, factory);
        circle1 = GeometricAnalysis.moveCircle(half, to, factory);
    }

    public Vector3<T> translate(Vector3<T> p) {
        if (circle0 == null || circle1 == null || circle0.math.isNaN(circle0.radius)
                || circle1.math.isNaN(circle1.radius)) {
            return p;
        } else {
            return circle1.reflect(circle0.reflect(p));
        }
    }
}

interface HyperbolicCircleFactory<T extends Number> {
    Circle<T> create(Vector3<T> center, Vector3<T> point);
}

class HyperbolicCircle<T extends Number> extends Circle<T> {

    HyperbolicCircle(Vector3<T> center, Vector3<T> point) {
        super(center.zero(), center.math.fromDouble(0.0)); // Temporary initialization

        boolean isColinear = GeometricAnalysis.isColinear(center, point);

        if (isColinear) {
            if (center.distanceTo(center.zero()).equals(center.math.fromDouble(0.0))) {
                this.center = center;
                this.radius = center.distanceTo(point);
            } else {
                Vector3<T> m = center.plus(new Circle<T>(center.zero(), center.math.fromDouble(1.0)).reflect(center))
                        .mul(center.math.fromDouble(0.5));
                Circle<T> c2 = GeometricAnalysis.twoPointCircle(m, center);
                Vector3<T> bpp = c2.reflect(point);
                Vector3<T> mid = point.plus(bpp).mul(center.math.fromDouble(0.5));
                Circle<T> hCircle = GeometricAnalysis.twoPointCircle(mid, point);
                this.center = hCircle.center;
                this.radius = hCircle.radius;
            }
        } else {
            Circle<T> circle = GeometricAnalysis.hyperbolicLine(center, point);
            Vector3<T> pointT = point.minus(circle.center);
            // The tangent of the point
            pointT = new Vector3<T>(center.math.negate(pointT.z), pointT.y, pointT.x, center.math);
            this.center = GeometricAnalysis.lineLineIntersection(center.zero(), center, point, point.plus(pointT));
            this.radius = this.center.distanceTo(point);
        }
    }
}

class HyperbolicCircleV2<T extends Number> extends Circle<T> {

    HyperbolicCircleV2(Vector3<T> center, Vector3<T> point) {
        super(center.zero(), point.math.fromDouble(0.0)); // Temporary initialization

        this.center = center.mul(
                math.subtract(point.dot(point), math.fromDouble(1.0))).div(
                        math.subtract(
                                math.subtract(
                                        math.multiply(center.dot(point), math.fromDouble(2.0)),
                                        center.dot(center)),
                                math.fromDouble(1.0)));
        this.radius = this.center.distanceTo(point);
    }
}

// Quadruple qerror_sum = new Quadruple(0.0);
// Quadruple qzerror_sum = new Quadruple(0.0);
// Double derror_sum = 0.0;
// Double dzerror_sum = 0.0;
// Float ferror_sum = 0.0f;
// Float fzerror_sum = 0.0f;
// int zeros = 0;
// int q_zeros = 0;
// int qz_zeros = 0;
// int d_zeros = 0;
// int dz_zeros = 0;
// int f_zeros = 0;
// int fz_zeros = 0;
// int f_nans = 0;
// Quadruple qmax_error = new Quadruple(0.0);
// Quadruple qzmax_error = new Quadruple(0.0);
// Double dmax_error = 0.0;
// Double dzmax_error = 0.0;
// Float fmax_error = 0.0f;
// Float fzmax_error = 0.0f;

class ErrorStats<T extends Number> {
    public T errorSum;
    public T maxError;
    public int zeroCount;
    public int nanCount;
    private final SimpleMath<T> math;

    public ErrorStats(SimpleMath<T> math) {
        this.math = math;
        this.errorSum = math.fromDouble(0.0);
        this.maxError = math.fromDouble(0.0);
        this.zeroCount = 0;
        this.nanCount = 0;
    }

    public void reset() {
        this.errorSum = math.fromDouble(0.0);
        this.maxError = math.fromDouble(0.0);
        this.zeroCount = 0;
        this.nanCount = 0;
    }

}

class ErrorSummary {
    public ErrorStats<Quadruple> qStats;
    public ErrorStats<Quadruple> qzStats;
    public ErrorStats<Quadruple> qStatsV2;
    public ErrorStats<Quadruple> qzStatsV2;
    public ErrorStats<Double> dStats;
    public ErrorStats<Double> dzStats;
    public ErrorStats<Double> dStatsV2;
    public ErrorStats<Double> dzStatsV2;
    public ErrorStats<Float> fStats;
    public ErrorStats<Float> fzStats;
    public ErrorStats<Float> fStatsV2;
    public ErrorStats<Float> fzStatsV2;

    public ErrorSummary() {
        this.qStats = new ErrorStats<Quadruple>(Vector3.QUADRUPLE_MATH);
        this.qzStats = new ErrorStats<Quadruple>(Vector3.QUADRUPLE_MATH);
        this.qStatsV2 = new ErrorStats<Quadruple>(Vector3.QUADRUPLE_MATH);
        this.qzStatsV2 = new ErrorStats<Quadruple>(Vector3.QUADRUPLE_MATH);
        this.dStats = new ErrorStats<Double>(Vector3.DOUBLE_MATH);
        this.dzStats = new ErrorStats<Double>(Vector3.DOUBLE_MATH);
        this.dStatsV2 = new ErrorStats<Double>(Vector3.DOUBLE_MATH);
        this.dzStatsV2 = new ErrorStats<Double>(Vector3.DOUBLE_MATH);
        this.fStats = new ErrorStats<Float>(Vector3.FLOAT_MATH);
        this.fzStats = new ErrorStats<Float>(Vector3.FLOAT_MATH);
        this.fStatsV2 = new ErrorStats<Float>(Vector3.FLOAT_MATH);
        this.fzStatsV2 = new ErrorStats<Float>(Vector3.FLOAT_MATH);
    }
}

public class GeometricAnalysis {
    public static <T extends Number> boolean isColinear(Vector3<T> p1, Vector3<T> p2) {
        SimpleMath<T> math = p1.math;
        return math.isZeroApprox(math.subtract(math.multiply(p1.x, p2.z), math.multiply(p1.z, p2.x)));
    }

    public static <T extends Number> Circle<T> twoPointCircle(Vector3<T> o, Vector3<T> p) {
        return new Circle<T>(o, o.distanceTo(p));
    }

    public static <T extends Number> Circle<T> createHyperbolicCircle(Vector3<T> center, Vector3<T> point) {
        return new HyperbolicCircle<T>(center, point);
    }

    public static <T extends Number> Circle<T> createHyperbolicCircleV2(Vector3<T> center, Vector3<T> point) {
        return new HyperbolicCircleV2<T>(center, point);
    }

    public static <T extends Number> Circle<T> hyperbolicLine(Vector3<T> p1, Vector3<T> p2) {
        Vector3<T> aToAr = new Circle<T>(p1.zero(), p1.math.fromDouble(1.0)).reflect(p1).minus(p1);
        Vector3<T> aToB = p2.minus(p1);

        return twoPointCircle(
                lineLineIntersection(
                        aToAr.mul(p1.math.fromDouble(0.5)).plus(p1),
                        aToAr.mul(p1.math.fromDouble(0.5)).plus(p1).plus(p1.up().cross(aToAr)),
                        aToB.mul(p1.math.fromDouble(0.5)).plus(p1),
                        aToB.mul(p1.math.fromDouble(0.5)).plus(p1).plus(p1.up().cross(aToB))),
                p2);
    }

    public static <T extends Number> Vector3<T> lineLineIntersection(Vector3<T> a1, Vector3<T> a2, Vector3<T> b1,
            Vector3<T> b2) {
        Vector3<T> a = a2.minus(a1);
        Vector3<T> b = b2.minus(b1);
        Vector3<T> c = b1.minus(a1);

        Vector3<T> xp1 = c.cross(b);
        Vector3<T> xp2 = a.cross(b);

        return a1.plus(a.mul(a1.math.divide(xp1.dot(xp2), xp2.lengthSquared())));
    }

    private static <T extends Number> T hsign(T x, SimpleMath<T> math) {
        return math.compareTo(x, math.fromDouble(0.0)) < 0 ? math.fromDouble(-1.0) : math.fromDouble(1.0);
    }

    public static <T extends Number> IntersectionResult<T> lineCircleIntersection(Vector3<T> o, Vector3<T> dir,
            Circle<T> circle) {
        SimpleMath<T> math = o.math;
        T dX = dir.x;
        T dY = dir.z;
        Vector3<T> x = o.minus(circle.center);
        Vector3<T> y = x.plus(dir);
        T dR2 = dir.lengthSquared();
        T D = math.subtract(
                math.multiply(x.x, y.z),
                math.multiply(x.z, y.x));
        T discriminant = math.subtract(
                math.multiply(math.multiply(circle.radius, circle.radius), dR2),
                math.multiply(D, D));

        if (math.compareTo(discriminant, math.fromDouble(0.0)) < 0) {
            return null;
        }

        T s = math.sqrt(discriminant);

        return new IntersectionResult<T>(
                new Vector3<T>(
                        math.add(
                                math.divide(math.add(math.multiply(D, dY),
                                        math.multiply(math.multiply(hsign(dY, math), dX), s)), dR2),
                                circle.center.x),
                        math.fromDouble(0.0),
                        math.add(
                                math.divide(math.add(math.multiply(math.negate(D), dX), math.multiply(math.abs(dY), s)),
                                        dR2),
                                circle.center.z),
                        math),
                new Vector3<T>(
                        math.add(
                                math.divide(math.subtract(math.multiply(D, dY),
                                        math.multiply(math.multiply(hsign(dY, math), dX), s)), dR2),
                                circle.center.x),
                        math.fromDouble(0.0),
                        math.add(
                                math.divide(math.subtract(math.multiply(math.negate(D), dX),
                                        math.multiply(math.abs(dY), s)), dR2),
                                circle.center.z),
                        math));
    }

    public static <T extends Number> IntersectionResult<T> circleCircleIntersection(Circle<T> circle1,
            Circle<T> circle2) {
        SimpleMath<T> math = circle1.math;
        Vector3<T> ab = circle1.center.minus(circle2.center);
        Vector3<T> n = circle1.center.up().cross(ab).normalized();
        T dist = ab.length();
        T d2 = math.multiply(dist, dist);
        T ra2 = math.multiply(circle1.radius, circle1.radius);
        T rb2 = math.multiply(circle2.radius, circle2.radius);
        T scale1 = math.divide(
                math.add(math.subtract(rb2, ra2), d2),
                math.multiply(math.fromDouble(2.0), dist));
        T scale2 = math.sqrt(math.subtract(rb2, math.multiply(scale1, scale1)));

        return new IntersectionResult<T>(
                ab.normalized().mul(scale1).plus(circle2.center).plus(n.mul(scale2)),
                ab.normalized().mul(scale1).plus(circle2.center).minus(n.mul(scale2)));
    }

    public static <T extends Number> HyperbolicSegment<T> moveCircle(Vector3<T> p1, Vector3<T> p2,
            HyperbolicCircleFactory<T> factory) {
        Circle<T> hc1 = factory.create(p1, p2);
        Circle<T> hc2 = factory.create(p2, p1);

        IntersectionResult<T> result = circleCircleIntersection(hc1, hc2);
        HyperbolicSegment<T> segment = new HyperbolicSegment<T>(result.intersection1, result.intersection2);

        if (segment.isColinear) {
            // System.out.println("Warning: Segment is colinear");
        }
        return segment;
    }

    public static <T extends Number> Vector3<T> randomUnitDiscPoint(SimpleMath<T> math) {
        T r = math.fromDouble(Math.random());
        double theta = Math.random() * 2 * Math.PI;
        return new Vector3<T>(
                math.multiply(r, math.fromDouble(Math.cos(theta))),
                math.fromDouble(0.0),
                math.multiply(r, math.fromDouble(Math.sin(theta))),
                math);
    }

    public static <T extends Number> Vector3<T> zeroToPointTransform(Vector3<T> p0, Vector3<T> p1,
            HyperbolicCircleFactory<T> factory) {
        // p0 - point to translate
        // p1 - target of translation
        final SimpleMath<T> math = p0.math;

        // Midpoints (OBS, the small distance 0.001 is necessary!!!)
        Vector3<T> rn = p1.minus(p0).cross(p1.up()).normalized().mul(math.fromDouble(0.001));
        Vector3<T> r0 = p0.plus(p1).mul(math.fromDouble(0.5));
        Vector3<T> r1 = r0.plus(rn);
        Vector3<T> r2 = r0.minus(rn);

        // transformation from p0 to p1
        HyperbolicSegment<T> move0P1 = moveCircle(p0, r0, factory);
        HyperbolicSegment<T> move0P2 = moveCircle(r0, p1, factory);

        HyperbolicSegment<T> move1P1 = moveCircle(p0, r1, factory);
        HyperbolicSegment<T> move1P2 = moveCircle(r1, p1, factory);

        HyperbolicSegment<T> move2P1 = moveCircle(p0, r2, factory);
        HyperbolicSegment<T> move2P2 = moveCircle(r2, p1, factory);

        // Construct the domain (circle) on which the solution exists on
        Vector3<T> z1 = p1.zero();
        Vector3<T> z2 = p1.zero();
        Vector3<T> z3 = p1.zero();

        if (areValidMoveCircles(move0P1, move0P2)) {
            z1 = move0P2.reflect(move0P1.reflect(p1.zero()));
        }

        if (areValidMoveCircles(move1P1, move1P2)) {
            z2 = move1P2.reflect(move1P1.reflect(p1.zero()));
        }

        if (areValidMoveCircles(move2P1, move2P2)) {
            z3 = move2P2.reflect(move2P1.reflect(p1.zero()));
        }

        Circle<T> circle = Circle.threePointCircle(z1, z2, z3);
        Vector3<T> transform;

        if (circle == null) {
            transform = p0.zero();
        } else {
            Vector3<T> S = circle.center.minus(p0.mul(math.divide(circle.radius, p0.length())));

            transform = S;

            if (math.compareTo(circle.center.length(), math.fromDouble(1.0)) > 0) {
                transform = p0.zero();
            }
        }

        return transform;

    }

    private static <T extends Number> boolean areValidMoveCircles(HyperbolicSegment<T> seg1,
            HyperbolicSegment<T> seg2) {
        return !seg1.isColinear && !seg2.isColinear;
    }

    private static <T extends Number> T transformError(Vector3<T> p0, Vector3<T> p1,
            HyperbolicCircleFactory<T> factory) {
        return (new TransformHB<T>(p0, p1, factory)).translate(p0).minus(p1).length();
    }

    private static <T extends Number> T transformFromZeroError(Vector3<T> p0, Vector3<T> p1,
            HyperbolicCircleFactory<T> factory) {
        return (new TransformHB<T>(p0.zero(), zeroToPointTransform(p0, p1, factory), factory)).translate(p0).minus(p1)
                .length();
    }

    private static boolean evaluateTransforms(Vector3<Quadruple> q0, Vector3<Quadruple> q1, ErrorSummary summary) {
        // Compute error metrics
        Quadruple qerror = transformError(q0, q1, GeometricAnalysis::createHyperbolicCircle);
        Quadruple qzerror = transformFromZeroError(q0, q1, GeometricAnalysis::createHyperbolicCircle);
        Quadruple qerrorV2 = transformError(q0, q1, GeometricAnalysis::createHyperbolicCircleV2);
        Quadruple qzerrorV2 = transformFromZeroError(q0, q1, GeometricAnalysis::createHyperbolicCircleV2);

        Vector3<Double> d0 = new Vector3<Double>(q0.x.doubleValue(), q0.y.doubleValue(), q0.z.doubleValue(),
                Vector3.DOUBLE_MATH);
        Vector3<Double> d1 = new Vector3<Double>(q1.x.doubleValue(), q1.y.doubleValue(), q1.z.doubleValue(),
                Vector3.DOUBLE_MATH);
        Double derror = transformError(d0, d1, GeometricAnalysis::createHyperbolicCircle);
        Double dzerror = transformFromZeroError(d0, d1, GeometricAnalysis::createHyperbolicCircle);
        Double derrorV2 = transformError(d0, d1, GeometricAnalysis::createHyperbolicCircleV2);
        Double dzerrorV2 = transformFromZeroError(d0, d1, GeometricAnalysis::createHyperbolicCircleV2);

        Vector3<Float> f0 = new Vector3<Float>(q0.x.floatValue(), q0.y.floatValue(), q0.z.floatValue(),
                Vector3.FLOAT_MATH);
        Vector3<Float> f1 = new Vector3<Float>(q1.x.floatValue(), q1.y.floatValue(), q1.z.floatValue(),
                Vector3.FLOAT_MATH);
        Float ferror = transformError(f0, f1, GeometricAnalysis::createHyperbolicCircle);
        Float fzerror = transformFromZeroError(f0, f1, GeometricAnalysis::createHyperbolicCircle);
        Float ferrorV2 = transformError(f0, f1, GeometricAnalysis::createHyperbolicCircleV2);
        Float fzerrorV2 = transformFromZeroError(f0, f1, GeometricAnalysis::createHyperbolicCircleV2);

        // Validate results
        if (ferror.isNaN()) {
            System.out.println("Coordinate: " + f0 + " to " + f1 + " caused NaN error");
            summary.fStats.nanCount++;
            return false;
        }
        if (fzerror.isNaN()) {
            System.out.println("Coordinate: " + f0 + " to " + f1 + " caused NaN error");
            summary.fzStats.nanCount++;
            return false;
        }
        if (qerror.equals(new Quadruple(0.0)))
            summary.qStats.zeroCount++;
        if (qzerror.equals(new Quadruple(0.0)))
            summary.qzStats.zeroCount++;
        if (qerrorV2.equals(new Quadruple(0.0)))
            summary.qStatsV2.zeroCount++;
        if (qzerrorV2.equals(new Quadruple(0.0)))
            summary.qzStatsV2.zeroCount++;
        if (derror.equals(0.0))
            summary.dStats.zeroCount++;
        if (dzerror.equals(0.0))
            summary.dzStats.zeroCount++;
        if (derrorV2.equals(0.0))
            summary.dStatsV2.zeroCount++;
        if (dzerrorV2.equals(0.0))
            summary.dzStatsV2.zeroCount++;
        if (ferror.equals(0.0f))
            summary.fStats.zeroCount++;
        if (fzerror.equals(0.0f))
            summary.fzStats.zeroCount++;
        if (ferrorV2.equals(0.0f))
            summary.fStatsV2.zeroCount++;
        if (fzerrorV2.equals(0.0f))
            summary.fzStatsV2.zeroCount++;
        if (qerror.equals(new Quadruple(0.0)) || qzerror.equals(new Quadruple(0.0)) ||
                qerrorV2.equals(new Quadruple(0.0)) || qzerrorV2.equals(new Quadruple(0.0)) ||
                derror.equals(0.0) || dzerror.equals(0.0) ||
                derrorV2.equals(0.0) || dzerrorV2.equals(0.0) ||
                ferror.equals(0.0f) || fzerror.equals(0.0f) ||
                ferrorV2.equals(0.0f) || fzerrorV2.equals(0.0f)) {
            return false;
        }

        // Gather statistics
        summary.qStats.errorSum = Quadruple.add(summary.qStats.errorSum, qerror);
        summary.qzStats.errorSum = Quadruple.add(summary.qzStats.errorSum, qzerror);
        summary.qStatsV2.errorSum = Quadruple.add(summary.qStatsV2.errorSum, qerrorV2);
        summary.qzStatsV2.errorSum = Quadruple.add(summary.qzStatsV2.errorSum, qzerrorV2);
        summary.dStats.errorSum += derror;
        summary.dzStats.errorSum += dzerror;
        summary.dStatsV2.errorSum += derrorV2;
        summary.dzStatsV2.errorSum += dzerrorV2;
        summary.fStats.errorSum += ferror;
        summary.fzStats.errorSum += fzerror;
        summary.fStatsV2.errorSum += ferrorV2;
        summary.fzStatsV2.errorSum += fzerrorV2;
        summary.qStats.maxError = Quadruple.max(summary.qStats.maxError, qerror);
        summary.qzStats.maxError = Quadruple.max(summary.qzStats.maxError, qzerror);
        summary.qStatsV2.maxError = Quadruple.max(summary.qStatsV2.maxError, qerrorV2);
        summary.qzStatsV2.maxError = Quadruple.max(summary.qzStatsV2.maxError, qzerrorV2);
        summary.dStats.maxError = Math.max(summary.dStats.maxError, derror);
        summary.dzStats.maxError = Math.max(summary.dzStats.maxError, dzerror);
        summary.dStatsV2.maxError = Math.max(summary.dStatsV2.maxError, derrorV2);
        summary.dzStatsV2.maxError = Math.max(summary.dzStatsV2.maxError, dzerrorV2);
        summary.fStats.maxError = Math.max(summary.fStats.maxError, ferror);
        summary.fzStats.maxError = Math.max(summary.fzStats.maxError, fzerror);
        summary.fStatsV2.maxError = Math.max(summary.fStatsV2.maxError, ferrorV2);
        summary.fzStatsV2.maxError = Math.max(summary.fzStatsV2.maxError, fzerrorV2);

        return true;
    }

    private static void printStats(String label, ErrorStats<Float> fStats, ErrorStats<Double> dStats,
            ErrorStats<Quadruple> qStats, int n) {
        Float ferror = fStats.errorSum / n;
        Double derror = dStats.errorSum / n;
        Quadruple qerror = qStats.errorSum.divide(new Quadruple(n));
        System.out.println("");
        System.out.println(label);
        System.out.println("");
        System.out.println("$$");
        System.out.println("\\begin{array}{lcccc}");
        System.out.println(
                "\\text{Precision} & \\bar{\\lVert\\varepsilon\\rVert} & \\hat{\\lVert\\varepsilon\\rVert} & 0 & \\text{NaNs}\\\\");
        System.out.println("\\hline");
        System.out.println("\\text{single} & " + String.format("%.3e", ferror).replace("e", " \\cdot 10^{") + "} & "
                + String.format("%.3e", fStats.maxError).replace("e", " \\cdot 10^{") + "} & " + fStats.zeroCount
                + " & " + fStats.nanCount
                + "\\\\");
        System.out.println("\\text{double} & " + String.format("%.3e", derror).replace("e", " \\cdot 10^{") + "} & "
                + String.format("%.3e", dStats.maxError).replace("e", " \\cdot 10^{") + "} & " + dStats.zeroCount
                + "&\\\\");
        System.out.println("\\text{quadruple}^* & " + qerror.format("%.3e").replace("e", " \\cdot 10^{") + "} & "
                + qStats.maxError.format("%.3e").replace("e", " \\cdot 10^{") + "} & " + qStats.zeroCount + "&\\\\");
        System.out.println("\\end{array}");
        System.out.println("$$");
        System.out.println("");
    }

    public static void main(String[] args) {
        Locale.setDefault(Locale.UK);
        System.out.println("Geometric Analysis Module:");
        int n = 100000;
        int retries = 0;

        ErrorSummary summary = new ErrorSummary();
        ErrorSummary colinear = new ErrorSummary();

        for (int i = 0; i - retries < n; i++) {
            Vector3<Quadruple> q0 = randomUnitDiscPoint(Vector3.QUADRUPLE_MATH);
            Vector3<Quadruple> q1 = randomUnitDiscPoint(Vector3.QUADRUPLE_MATH);
            if (!evaluateTransforms(q0, q1, summary)) {
                retries++;
            }
        }

        retries = 0;

        for (int i = 0; i - retries < n; i++) {
            // Generate two colinear points by scaling a random direction
            Vector3<Quadruple> dir = randomUnitDiscPoint(Vector3.QUADRUPLE_MATH).normalized();
            double scale1 = Math.random();
            double scale2 = Math.random();
            Vector3<Quadruple> q0 = dir.mul(new Quadruple(scale1));
            Vector3<Quadruple> q1 = dir.mul(new Quadruple(scale2));
            if (!evaluateTransforms(q0, q1, colinear)) {
                retries++;
            }
        }

        printStats("Errors for " + n + " random points transformations $M_D(A,A,B,0)$:", summary.fStats, summary.dStats,
                summary.qStats, n);
        printStats("Errors for " + n + " random points transformations simplified $M_D(A,A,B,0)$:", summary.fStatsV2,
                summary.dStatsV2, summary.qStatsV2, n);
        printStats("Errors for " + n + " random points transformations $M_D(A,0,\\xi,0)$:", summary.fzStats,
                summary.dzStats, summary.qzStats, n);
        printStats("Errors for " + n + " random points transformations simplified $M_D(A,0,\\xi,0)$:",
                summary.fzStatsV2, summary.dzStatsV2, summary.qzStatsV2, n);

        printStats("Errors for " + n + " colinear points transformations $M_D(A,A,B,0)$:", summary.fStats,
                summary.dStats, summary.qStats, n);
        printStats("Errors for " + n + " colinear points transformations simplified $M_D(A,A,B,0)$:", summary.fStatsV2,
                summary.dStatsV2, summary.qStatsV2, n);
        printStats("Errors for " + n + " colinear points transformations $M_D(A,0,\\xi,0)$:", summary.fzStats,
                summary.dzStats, summary.qzStats, n);
        printStats("Errors for " + n + " colinear points transformations simplified $M_D(A,0,\\xi,0)$:",
                summary.fzStatsV2, summary.dzStatsV2, summary.qzStatsV2, n);
    }
}
