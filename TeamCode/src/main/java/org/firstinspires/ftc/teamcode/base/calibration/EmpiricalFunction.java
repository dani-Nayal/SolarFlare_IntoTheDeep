package org.firstinspires.ftc.teamcode.base.calibration;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.function.Function;

public class EmpiricalFunction implements Function<Double, Double> {
    private final ArrayList<NumberPair> f = new ArrayList<>();

    public void addDataPoint(double x, double y) {
        f.add(new NumberPair(x, y));
        f.sort((a, b) -> Double.compare(a.n1, b.n1));
    }

    public Double apply(Double x) {
        NumberPair lastNP = null;
        for(NumberPair np: f) {
            if(x == np.n1) {
                return np.n2;
            } else if(x > np.n1) {
                lastNP = np;
            } else {
                if(lastNP == null) {
                    return np.n2;
                }
                double w1 = (x-lastNP.n1) / (np.n1-lastNP.n1);
                return w1 * np.n2 + (1-w1) * lastNP.n2;
            }
        }
        return lastNP.n2;
    }

    @NonNull
    @Override
    public String toString() {
        return f.toString();
    }

    public static void main(String[] args) {
        EmpiricalFunction f = new EmpiricalFunction();
        f.addDataPoint(10.0,1);
        f.addDataPoint(9.0,0.9);
        f.addDataPoint(8.0, 0.8);
        f.addDataPoint(5.0, 0.5);
        f.addDataPoint(1.0, 0.1);

        System.out.println("f=" + f);

        System.out.println("f(10)=" + f.apply(10.0));
        System.out.println("f(9)="  + f.apply(9.0));
        System.out.println("f(8)="  + f.apply(8.0));
        System.out.println("f(7)="  + f.apply(7.0));
        System.out.println("f(6)="  + f.apply(6.0));
        System.out.println("f(5)="  + f.apply(5.0));
        System.out.println("f(4)="  + f.apply(4.0));
        System.out.println("f(3)="  + f.apply(3.0));
        System.out.println("f(2)="  + f.apply(2.0));
        System.out.println("f(1)="  + f.apply(1.0));
    }
}
