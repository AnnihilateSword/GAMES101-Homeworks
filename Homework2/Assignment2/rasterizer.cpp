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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f ab{_v[1] - _v[0]};
    Vector3f bc{_v[2] - _v[1]};
    Vector3f ca{_v[0] - _v[2]};
    // 这里我们不用考虑 z 轴，z-buffer 会处理
    ab.z() = 0, bc.z() = 0, ca.z() = 0;
    Vector3f ap{x-_v[0].x(), y-_v[0].y(), 0};
    Vector3f bp{x-_v[1].x(), y-_v[1].y(), 0};
    Vector3f cp{x-_v[2].x(), y-_v[2].y(), 0};

    return ((ab.cross(ap).z() > 0 && bc.cross(bp).z() > 0 && ca.cross(cp).z() > 0) ||
            (ab.cross(ap).z() < 0 && bc.cross(bp).z() < 0 && ca.cross(cp).z() < 0));
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
            vert.z() = -vert.z() * f1 + f2;
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

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    min_x = static_cast<int>(std::floor(min_x));
    min_y = static_cast<int>(std::floor(min_y));
    max_x = static_cast<int>(std::ceil(max_x));
    max_y = static_cast<int>(std::ceil(max_y));

    // 是否启用 MSAA 处理抗锯齿

    bool MSAA = true;
    std::vector<Vector2f> SamplePosBias
    {
        {0.25, 0.25},
        {0.25, 0.75},
        {0.75, 0.25},
        {0.75, 0.75}
    };

    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = min_x; x < max_x; x++)
    {
        for (int y = min_y; y < max_y; y++)
        {
            if (MSAA)
            {
                int MSAASampleCount = 0;
                for (int i = 0; i < 4; i++)
                {
                    if (insideTriangle(x + SamplePosBias[i][0], y + SamplePosBias[i][1], t.v))
                    {
                        float MinDepth = FLT_MAX;

                        // 这里提示一下：我们通过任意一点都能计算出重心坐标，这里任意一点要和我们的采样点对应

                        auto[alpha, beta, gamma] = computeBarycentric2D(x + SamplePosBias[i][0], y + SamplePosBias[i][1], t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        MinDepth = std::min(z_interpolated, MinDepth);

                        // 每个采样点都要记录深度和颜色信息
                        if (super_depth_buf[get_super_index(2*x + i%2, 2*y + i/2)] > MinDepth)
                        {
                            super_depth_buf[get_super_index(2*x + i%2, 2*y + i/2)] = MinDepth;
                            super_frame_buf[get_super_index(2*x + i%2, 2*y + i/2)] = t.getColor();
                        }

                        MSAASampleCount++;
                    }
                }
                if (MSAASampleCount > 0)
                {
                    Vector3f point;
                    point << x, y, 0;
                    // 采样点颜色取平均
                    Vector3f color = (super_frame_buf[get_super_index(2*x, 2*y)] + super_frame_buf[get_super_index(2*x + 1, 2*y)] +
                                super_frame_buf[get_super_index(2*x, 2*y + 1)] + super_frame_buf[get_super_index(2*x + 1, 2*y + 1)]) / 4.0f;
                    set_pixel(point, color);
                }
            }
            else
            {
                if (insideTriangle(x + 0.5f, y + 0.5f, t.v))
                {
                    float MinDepth = FLT_MAX;

                    // If so, use the following code to get the interpolated z value.
                    auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y + 0.5f, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    MinDepth = std::min(z_interpolated, MinDepth);

                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    if (depth_buf[get_index(x, y)] > MinDepth)
                    {
                        depth_buf[get_index(x, y)] = MinDepth;
                        Vector3f point;
                        point << x, y, MinDepth;
                        set_pixel(point, t.getColor());
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
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y)
{
    return (height*2-1-y)*width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on