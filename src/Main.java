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
                .withEnterVelocity(1)
                .withExitVelocity(2)
                .withDestination(10, 2, 0.3)
                .generateSequence(0.005)
                .generateProfiles(16);
        final int N = path.size();
        Mat dataX = new Mat(1, N, CvType.CV_64F);
        Mat dataY = new Mat(1, N, CvType.CV_64F);
        double dx = path.length()/N;
        for(int i=0; i < N; i++) {
            double x = dx*i;
            dataX.put(0, i, x);
            dataY.put(0, i, -path.getVelocity(i));
            dataY.put(0, i, -path.getAlpha(i)*10);
            //dataY.put(0, i, -path.getPosition(i));
            if(i==0)
                dataY.put(0, i, 0);
            else
                dataY.put(0, i, 100*(path.profileLeft[i-1][0]-path.profileLeft[i][0]));
        }
        Plot2d plot = Plot2d.create(dataX, dataY);
        plot.setPlotSize(1200, 900);
        plot.setMinX(-2);
        plot.setMaxX(15);
        plot.setMinY(-12);
        plot.setMaxY(12);
        Mat mplot = new Mat();
        plot.setPlotLineColor(new Scalar(0,255,0));
        plot.render(mplot);
        HighGui.imshow("Plot", mplot);
        HighGui.waitKey();
        HighGui.destroyAllWindows();
    }
}
