#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
        return control_points[0];

    std::vector<cv::Point2f> new_control_point;
    for (int i = 0; i < control_points.size() - 1; i++)
    {
        const cv::Point2f& point0 = control_points[i];
        const cv::Point2f& point1 = control_points[i + 1];

        new_control_point.emplace_back(point0 + t * (point1 - point0));
    }

    return recursive_bezier(new_control_point, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

	for (double t = 0.0; t <= 1.0; t += 0.001)
	{
        cv::Point2f point = recursive_bezier(control_points, t);

		window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        int x = std::floor(point.x);
        int y = std::floor(point.y);

        float offsetx = (point.x - x) > 0.5f ? 1.0f : -1.0f;
        float offsety = (point.y - y) > 0.5f ? 1.0f : -1.0f;

        std::vector<cv::Point2f> sample_points;
        sample_points.emplace_back(x + 0.5f, y + 0.5f);
        sample_points.emplace_back(x + 0.5f, y + offsety + 0.5f);
        sample_points.emplace_back(x + offsetx + 0.5f, y + 0.5f);
        sample_points.emplace_back(x + offsetx + 0.5f, y + offsety + 0.5f);

        cv::Point2f d = point - sample_points[0];
        float base = std::sqrt(d.x * d.x + d.y * d.y);

        for(cv::Point2f & sample_point : sample_points)
        {
            d = point - sample_point;
            float dis = std::sqrt(d.x * d.x + d.y * d.y);
            float color = window.at<cv::Vec3b>(point.y, point.x)[1];
            color = std::max((float)255, dis / base * color);
            window.at<cv::Vec3b>(sample_point.y, sample_point.x)[1] = color;
        }
	}
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
