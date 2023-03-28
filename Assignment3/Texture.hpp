//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#pragma warning(disable:4244)

class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
		if (u < 0) u = 0;
		if (u > 1) u = 1;
		if (v < 0) v = 0;
		if (v > 1) v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

	Eigen::Vector3f getColorBiLinear(float u, float v)
	{
		if (u < 0) u = 0;
		if (u > 1) u = 1;
		if (v < 0) v = 0;
		if (v > 1) v = 1;

		auto u_img = u * width;
		auto v_img = (1 - v) * height;

        int u_min = std::floor(u_img);
        int u_max = std::ceil(u_img);
		int v_min = std::floor(v_img);
		int v_max = std::ceil(v_img);

        auto color_tl = image_data.at<cv::Vec3b>(v_min, u_min);
        auto color_tr = image_data.at<cv::Vec3b>(v_min, u_max);
        float lerp_u = (u_img - u_min) / (u_max - u_min);
        float lerp_v = (v_img - v_min) / (v_max - v_min);

        auto color1 = color_tl + (color_tr - color_tl) * lerp_u;

		auto color_bl = image_data.at<cv::Vec3b>(v_max, u_min);
		auto color_br = image_data.at<cv::Vec3b>(v_max, u_max);
        auto color2 = color_bl + (color_br - color_bl) * lerp_u;

        auto color = color1 + (color2 - color1) * lerp_v;

		//auto color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

};
#endif //RASTERIZER_TEXTURE_H
