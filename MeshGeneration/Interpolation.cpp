#include<iostream>
#include<vector>
#include<gsl/gsl_spline.h>
#include<list>

struct DataPoint {
    double coordinate;
    double value;

    DataPoint(double x, double y) : coordinate(x), value(y) {}
};
void Normalize(std::list<double>& area)
{
    double temp = 0;
    for (auto& i : area)
    {
        i += temp;
        temp = i;
    }
    for (auto& i : area)
    {
        i = i / temp;
    }
}
class spline {
private:
    std::vector<DataPoint> data;
    gsl_interp_accel* acc = gsl_interp_accel_alloc();
    gsl_spline* gspline = nullptr;

public:
    spline(std::vector<DataPoint> data) : data(data) {}

    spline(std::list<double> coordinate, std::list<double> area)
    {
		auto it1 = coordinate.begin();
		auto it2 = area.begin();
        for (; it1 != coordinate.end() && it2 != area.end(); ++it1, ++it2)
        {
			data.push_back(DataPoint(*it1, *it2));
		}
    }

    gsl_spline* getSpline() {
		return gspline;
	}

    void computeCubicSpline() {
        size_t n = data.size();
        std::vector<double> x(n), y(n);
        for (size_t i = 0; i < n; ++i) {
            x[i] = data[i].coordinate;
            y[i] = data[i].value;
        }

        gspline = gsl_spline_alloc(gsl_interp_cspline, n);
        gsl_spline_init(gspline, x.data(), y.data(), n);

    }

    double evaluateSpline(double x) {
        return gsl_spline_eval(gspline, x, acc);
    }

    double find_x_for_y(double target_y) {

        double x = data[0].coordinate;
        double tolerance = 1e-6;
        int max_iterations = 1000;

        for (int i = 0; i < max_iterations; ++i) {
            double y = evaluateSpline(x);
            double derivative = (evaluateSpline(x + tolerance) - evaluateSpline(x)) / tolerance;
            x -= (y - target_y) / derivative;
            if (std::abs(y - target_y) < tolerance) {
                break;
            }
        }

        return x;
    }

};




