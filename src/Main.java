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
        CubicPath path = new CubicPath(2, 4.5)
                .withEnterVelocity(0)
                .withExitVelocity(0)
                .withDuration(0.01)
                .generateSequence(3, 1.5, 0.0)
                .generateProfiles(1);
        final int N = path.getSize();
        Mat dataX = new Mat(1, N, CvType.CV_64F);
        Mat dataY = new Mat(1, N, CvType.CV_64F);
        double dx = path.getLength()/N;
        for(int i=0; i < N; i++) {
            double x = dx*i;
            dataX.put(0, i, x);
            dataY.put(0, i, -path.getVelocity(i));
            dataY.put(0, i, -path.getAlpha(i));
            dataY.put(0, i, -path.getPosition(i));
            dataY.put(0, i, path.profileLeft[i][1]-path.profileRight[i][1]);
        }
        Plot2d plot = Plot2d.create(dataX, dataY);
        plot.setPlotSize(1200, 900);
        plot.setMinX(-2);
        plot.setMaxX(5);
        plot.setMinY(-5);
        plot.setMaxY(5);
        Mat mplot = new Mat();
        plot.setPlotLineColor(new Scalar(0,255,0));
        plot.render(mplot);
        HighGui.imshow("Plot", mplot);
        HighGui.waitKey();
        HighGui.destroyAllWindows();
    }
}
