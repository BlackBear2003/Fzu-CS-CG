// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>


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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector2f v0, v1, v2;
    v0.x() = _v[0].x();
    v0.y() = _v[0].y();
    v1.x() = _v[1].x();
    v1.y() = _v[1].y();
    v2.x() = _v[2].x();
    v2.y() = _v[2].y();

    float L0 = -(x-v0.x())*(v1.y()-v0.y()) + (y-v0.y())*(v1.x()-v0.x());
    float L1 = -(x-v1.x())*(v2.y()-v1.y()) + (y-v1.y())*(v2.x()-v1.x());
    float L2 = -(x-v2.x())*(v0.y()-v2.y()) + (y-v2.y())*(v0.x()-v2.x());

    return (L0>0 && L1>0 && L2>0) || (L0<0 && L1<0 && L2<0);

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
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
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

        ss_rasterize_triangle(t);
    }

    // render the depth and color of the pixel
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            Vector3f pixel_color(0.0f, 0.0f, 0.0f);

            for (int ss_x = 0; ss_x < ss_rate; ss_x++) {
                for (int ss_y = 0; ss_y < ss_rate; ss_y++) {
                    int index = get_index(x*ss_rate+ss_x, y*ss_rate+ss_y);
                    pixel_color += ss_frame_buf[index];
                }
            }

            pixel_color /= ss_rate * ss_rate;
            Vector3f p;
            p.x() = x;
            p.y() = y;
            set_pixel(p, pixel_color);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    int min_x = 1000, min_y = 1000, max_x = 0, max_y = 0;

    for(auto e : v) {
        min_x = fmin(min_x, e.x());
        min_y = fmin(min_y, e.y());
        max_x = fmax(max_x, e.x());
        max_y = fmax(max_y, e.y());
    }

    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            if(insideTriangle(x+0.5, y+0.5, t.v)) {
                float alpha, beta, gamma;
                std::tie(alpha, beta, gamma) = computeBarycentric2D(x+0.5, y+0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                int index = get_index(x, y);
                if(z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    Vector3f p;
                    p.x() = x;
                    p.y() = y;
                    p.z() = z_interpolated;
                    set_pixel(p, t.getColor());
                }
            }
        }
    }
}

//Screen space super-sampling rasterization
void rst::rasterizer::ss_rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    int min_x = 1000, min_y = 1000, max_x = 0, max_y = 0;

    for(auto e : v) {
        min_x = fmin(min_x, e.x());
        min_y = fmin(min_y, e.y());
        max_x = fmax(max_x, e.x());
        max_y = fmax(max_y, e.y());
    }

    for (int x = min_x; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {
            for (int ss_x = 0; ss_x < ss_rate; ss_x++) {
                for (int ss_y = 0; ss_y < ss_rate; ss_y++) {
                    float sample_x = x + (ss_x + 0.5) / ss_rate;
                    float sample_y = y + (ss_y + 0.5) / ss_rate;

                    if(insideTriangle(sample_x, sample_y, t.v)) {
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(sample_x, sample_y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int ss_index = get_index(x*ss_rate + ss_x, y*ss_rate + ss_y);
                        if (z_interpolated < ss_depth_buf[ss_index]) {
                            
                            ss_depth_buf[ss_index] = z_interpolated;
                            
                            ss_frame_buf[ss_index] = t.getColor();
                        }
                    }
                }
            }
        }
    }
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

    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(ss_frame_buf.begin(), ss_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(ss_depth_buf.begin(), ss_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    ss_depth_buf.resize(ss_rate * w * ss_rate * h);
    ss_frame_buf.resize(ss_rate * w * ss_rate * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return y * width * ss_rate + x;
}


void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on