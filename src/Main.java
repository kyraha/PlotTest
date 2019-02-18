import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.plot.Plot2d;
import java.util.ArrayList;
import java.util.List;

public class Main {

    public static void main(String[] args)
    {
        System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
/*        List<Double> xList = new ArrayList<>();
        xList.add(0.0);
        xList.add(2.0);
        xList.add(6.0);
        xList.add(8.0);
        List<Double> yList = new ArrayList<>();
        yList.add(0.0);
        yList.add(2.0);
        yList.add(2.5);
        yList.add(2.0);
        SplineInterpolator spline = SplineInterpolator.createMonotoneCubicSpline(xList, yList); */
        CubicPath path = new CubicPath(8, 2, -.5, 10, 10).generate(0.01);
        final int N = path.size();
        Mat dataX = new Mat(1, N, CvType.CV_64F);
        Mat dataY = new Mat(1, N, CvType.CV_64F);
        for(int i=0; i < N; i++) {
            double x = path.length()*i/N;
            dataX.put(0, i, x);
            //data.put(0, i,-spline.interpolate(8.0*i/N));
            dataY.put(0, i, -path.getAlpha(i)*100);
        }
        Plot2d plot = Plot2d.create(dataX, dataY);
        plot.setMinX(-2);
        plot.setMaxX(12);
        plot.setMinY(-15);
        plot.setMaxY(15);
        Mat mplot = new Mat();
        plot.setPlotLineColor(new Scalar(0,255,0));
        plot.render(mplot);
        HighGui.imshow("Plot", mplot);
        HighGui.waitKey();
        HighGui.destroyAllWindows();
    }
}
