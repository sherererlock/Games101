// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight) {
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f P(x , y , 1.0f);

    bool p1 = (P - _v[0]).cross(_v[1] - _v[0]).z() > 0;
    bool p2 = (P - _v[1]).cross(_v[2] - _v[1]).z() > 0;
    bool p3 = (P - _v[2]).cross(_v[0] - _v[2]).z() > 0;

    return p1 == p2 && p2 == p3;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w(); //仅仅是为了比较深度大小
        }

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.w();
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}


void rst::rasterizer::msaa(float x, float y, const Triangle& t, const std::array<Vector4f, 3>& v)
{
    int totalSampleCount = msaa_w * msaa_h;

    float w = 1.0f / msaa_w;
    float h = 1.0f / msaa_h;

    int insideSampleCount = 0;
    for (int j = 0; j < msaa_w; j++)
    {
        for (int i = 0; i < msaa_h; i++)
        {
            float px = x + w * i + w * 0.5f;
            float py = y + h * j + h * 0.5f;
            if (insideTriangle(px, py, t.v))
            {
                int buffer_idx = x * msaa_w + i + (y * msaa_h + j) * width * msaa_w;
                //std::cout << px << " " << py << " " << sample_idx << std::endl;
				auto [alpha, beta, gamma] = computeBarycentric2D(px, py, t.v);
				float z = 1.0 / (alpha / v[0].z() + beta / v[1].z() + gamma / v[2].z());
				if (z > depth_buf[buffer_idx])
					continue;

                depth_buf[buffer_idx] = z;
                insideSampleCount++;
            }
        }
    }

	if (insideSampleCount > 0)
	{
		auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
		float z = 1.0 / (alpha / v[0].z() + beta / v[1].z() + gamma / v[2].z());

		Vector3f color = z * (alpha * t.color[0] / v[0].z() + beta * t.color[1] / v[1].z() + gamma * t.color[2] / v[2].z());
		color *= 255;
        color *= ((float)insideSampleCount / (float)totalSampleCount);
		mix_pixel(Vector3f(x, y, 1.0f), color);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    float minx = width, miny = height, maxx = 0, maxy = 0;

    for (int i = 0; i < v.size(); i++)
    {
        if (v[i].x() < minx)
            minx = v[i].x();
        if(v[i].x() > maxx)
            maxx = v[i].x();

		if (v[i].y() < miny)
			miny = v[i].y();
		if (v[i].y() > maxy)
			maxy = v[i].y();
    }

    for(int y = miny; y < maxy; y ++)
    {
        for (int x = minx; x < maxx; x ++)
        {
            msaa(x, y, t, v);
        }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    msaa_w = 4;
    msaa_h = 4;
    int sample_count = msaa_h * msaa_w;
    frame_buf.resize(w * h);
    depth_buf.resize(w * h * sample_count);
}

int rst::rasterizer::get_index(int x, int y, int sample_idx)
{
    return ((height-1-y)*width + x) * 4 + sample_idx;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::mix_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] += color;

}

// clang-format on