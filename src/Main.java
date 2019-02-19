import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.plot.Plot2d;

public class Main {

    public static void main(String[] args)
    {
        System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
        CubicPath path = new CubicPath(2, 3)
                .withEnterVelocity(2)
                .withExitVelocity(0)
                .withDestination(10, 2, 0.2)
                .generate(0.05);
        final int N = path.size();
        Mat dataX = new Mat(1, N, CvType.CV_64F);
        Mat dataY = new Mat(1, N, CvType.CV_64F);
        double dx = path.length()/N;
        for(int i=0; i < N; i++) {
            double x = dx*i;
            dataX.put(0, i, x);
            dataY.put(0, i, -path.getVelocity(i));
            //dataY.put(0, i, -path.getAlpha(i)*100);
        }
        Plot2d plot = Plot2d.create(dataX, dataY);
        plot.setMinX(-2);
        plot.setMaxX(12);
        plot.setMinY(-7);
        plot.setMaxY(7);
        Mat mplot = new Mat();
        plot.setPlotLineColor(new Scalar(0,255,0));
        plot.render(mplot);
        HighGui.imshow("Plot", mplot);
        HighGui.waitKey();
        HighGui.destroyAllWindows();
    }
}
