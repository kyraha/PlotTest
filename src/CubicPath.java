/*
 * Copyright (C) 2019 The ERRORS FRC Team 3130
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


import java.util.ArrayList;
import java.util.List;

/**
 * Performs spline like interpolation given a finish point and angle(tangent).
 * Starting point is always (0,0) with the direction straight along the X axis.
 * 
 */
public class CubicPath {

	private double a ,b;
	private double L, D, A;
	private double destination;
	private List<Double> mmPosition = new ArrayList<>();
	private List<Double> mmVelocity = new ArrayList<>();
	private List<Double> mmAlpha = new ArrayList<>();
	private double enterVelocity = 0;
	private double exitVelocity = 0;
	private double cruiseVelocity;
	private double maxAcceleration;
	private double dt;

	/**
	 * Creates a monotone cubic spline segment from origin to a given control point.
	 * The spline is guaranteed to end at the control point exactly at the given slope.
	 *
	 * @param x
	 *            The X component of the end point
	 * @param y
	 *            The Y component of the end point
	 * @param slope
	 *            The slope, tan of the angle, first derivative, at the end point
	 * @throws IllegalArgumentException
	 *             if the X or Y arrays are null, have different lengths or have fewer than 2 values.
	 */
	public CubicPath(double x, double y, double slope, double maxAcceleration, double cruiseVelocity) {
		if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(slope) || x <= 0) {
			throw new IllegalArgumentException("The end point has to be strictly positive");
		}
		L = x;
		D = y;
		A = slope;
		this.maxAcceleration = maxAcceleration;
		this.cruiseVelocity = cruiseVelocity;
		a = -2*D/(L*L*L) +A/(L*L);
		b = 3*D/(L*L) -A/L;
		destination = L * Math.sqrt(1 + Math.pow(3*a*L*L + 2*b*L, 2));
	}

	/**
	 * Interpolates the value of Y = f(X) for given X. Clamps X to the domain of the spline.
	 * 
	 * @param x
	 *            The X value.
	 * @return The interpolated Y = f(X) value.
	 */
	public double interpolate(double x) {
		// Handle the boundary cases.
		if (Double.isNaN(x)) {
			return x;
		}
		if (x <= 0.0) {
			return 0.0;
		}
		if (x >= L) {
			return D + A*(x - L);
		}
		return a*x*x*x + b*x*x;
	}

	public CubicPath generate(double deltaTime) {
		dt = deltaTime;
		if (Double.isNaN(maxAcceleration) || Double.isNaN(cruiseVelocity) || maxAcceleration <= 0 || cruiseVelocity <= 0) {
			throw new IllegalArgumentException("Max acceleration and cruise velocity must be positive");
		}
		double upTime = (cruiseVelocity - enterVelocity)/maxAcceleration;
		double dnPeriod = (cruiseVelocity - exitVelocity)/maxAcceleration;
		double upDistance = upTime*(enterVelocity + cruiseVelocity)/2.0;
		double dnDistance = dnPeriod*(exitVelocity + cruiseVelocity)/2.0;
		double cruiseDistance = destination - upDistance - dnDistance;
		double cruisePeriod = cruiseDistance/cruiseVelocity;
		double dnTime = upTime + cruisePeriod;
		double totalTime = upTime + cruisePeriod + dnPeriod;
		double s = 0.0, x = 0.0, oldTheta = 0.0;
		System.out.println("Total time: " + totalTime);
		for(double t = 0.0; t <= totalTime; t += dt) {
			double velocity;
			if(t < upTime) velocity = enterVelocity + maxAcceleration * t;
			else if(t > dnTime) velocity = exitVelocity + maxAcceleration * (totalTime - t);
			else velocity = cruiseVelocity;
			double dS = velocity * dt;
			// Main curve is y = a*x^3 + b*x^2 therefore its derivative is:
			double dydx = 3*a*x*x + 2*b*x;
			// dx is how much we move along the X axis:
			double dx = dS / Math.sqrt(1 + dydx*dydx);
			double theta = Math.atan(dydx);
			mmPosition.add(s);
			mmVelocity.add(velocity);
			mmAlpha.add(theta - oldTheta);
			System.out.println("X: "+x+"Theta: " + theta + ", dy/dx = " + dydx);
			oldTheta = theta;
			s += dS;
			x += dx;
		}
		System.out.println("Generation is done. Size: " + size());
		return this;
	}

	int size() {
		return mmPosition.size();
	}

	double length() {
		return destination;
	}

	double getPosition(int i) {
		return mmPosition.get(i);
	}

	double getAlpha(int i) {
		return mmAlpha.get(i);
	}

	double getVelocity(int i) {
		return mmVelocity.get(i);
	}
}
