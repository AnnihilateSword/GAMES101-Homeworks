#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    // Create the projection matrix for the given parameters.
    // Then return it.
    // Mortho
    // 2/(r-l)  0       0       -(r-l)/2
    // 0        2/(t-b) 0       -(t-b)/2
    // 0        0       2/(n-f) -(n-f)/2
    // 0        0       0       1
    // Mprojection => Mortho
    // n  0  0  0
    // 0  n  0  0
    // 0  0 n+f -nf
    // 0  0  1  0
	
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    eye_fov = eye_fov * MY_PI / 180;
    float t = zNear * tan(eye_fov/2);
    float r = t * aspect_ratio;
    float l = -r;
    float b = -t;
    // 正交变换矩阵
    Eigen::Matrix4f Mortho = Eigen::Matrix4f::Identity();
    Mortho << 2/(r-l), 0, 0, -(r-l)/2,
            0, 2/(t-b), 0, -(t-b)/2,
            0, 0, 2/(zNear-zFar), -(zNear-zFar)/2,
            0, 0, 0, 1;

    // 投影矩阵 => 正交矩阵
    Eigen::Matrix4f Mp2o = Eigen::Matrix4f::Identity();
    Mp2o << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear+zFar, -zNear*zFar,
            0, 0, 1, 0;

    // 因为 Opencv 坐标系的原因，绘制出来的三角形是倒着的，这里需要再乘一个矩阵

    Eigen::Matrix4f Mz(4, 4);
    Mz << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    projection = Mortho * Mp2o * Mz * projection;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
        // return_color = payload.texture->getColorBilinear(payload.tex_coords[0], payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    // 1. ambient
    Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        float r = (light.position - point).norm();
        float r_square = r * r;
        Vector3f l = (light.position - point).normalized();

        // 2. diffuse = kd * (l / r^2) * max(0, n·l)
        diffuse += kd.cwiseProduct(light.intensity / r_square) * std::max(0.0f, normal.dot(l));

        // 3. specular = ks * (l / r^2) * pow(max(0, n·h), p)
        // v 是着色点指向相机方向的向量

        Vector3f v = eye_pos - point;
        // h 是 l 和 v 的对角线单位向量
        Vector3f h = (l + v).normalized();
        specular += ks.cwiseProduct(light.intensity / r_square) * pow(std::max(0.0f, normal.dot(h)), p);
    }
    result_color = ambient + diffuse + specular;
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    // 1. ambient
    Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        
        float r = (light.position - point).norm();
        float r_square = r * r;
        Vector3f l = (light.position - point).normalized();

        // 2. diffuse = kd * (l / r^2) * max(0, n·l)
        diffuse += kd.cwiseProduct(light.intensity / r_square) * std::max(0.0f, normal.dot(l));

        // 3. specular = ks * (l / r^2) * pow(max(0, n·h), p)
        // v 是着色点指向相机方向的向量

        Vector3f v = eye_pos - point;
        // h 是 l 和 v 的对角线单位向量
        Vector3f h = (l + v).normalized();
        specular += ks.cwiseProduct(light.intensity / r_square) * pow(std::max(0.0f, normal.dot(h)), p);
    }
    result_color = ambient + diffuse + specular;
    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    Vector3f t = (payload.T).normalized();
    Vector3f b = normal.cross(t).normalized();
    Matrix3f TBN{
        {t.x(), b.x(), normal.x()},
        {t.y(), b.z(), normal.y()},
        {t.z(), b.z(), normal.z()}
    };

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // kh kn 表示纹理定义的法线对真实物体法线的影响系数（程度）
    // 然后这里注意了，因为这个 Demo 问题，这个名称如果不清楚会很容易误解
    // 我们测试 凹凸贴图 的时候，给 texture 传入的是 hmap（高度图），而不是我们惯性思维上的普通纹理贴图

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f/h).norm() - payload.texture->getColor(u, v).norm());

    // 将这个坐标写成(s,v,u)移动在p点移动1个单位s，u的变化是dp/du,v的变化是dp/dv，然后得到一个点，
    // 这个点减去p点（这里是视频中的着色点）就是一个p到该点的向量，然后叉乘这个向量得到法向量.
    // u v切线的方向向量的叉乘就是法线方向
    // 其实就是 uv 方向的切线叉乘 => (1, 0, dU) 叉乘 (0, 1, dV)
    Eigen::Vector3f ln = Eigen::Vector3f{-dU, -dV, 1.0f};
    point = point + kn * normal.cwiseProduct(payload.texture->getColor(u,v));
    normal = (TBN * ln).normalized();

    // 位移贴图
    Eigen::Vector3f result_color = {0, 0, 0};

    // 1. ambient
    Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

    Eigen::Vector3f diffuse = {0, 0, 0};
    Eigen::Vector3f specular = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        float r = (light.position - point).norm();
        float r_square = r * r;
        Vector3f l = (light.position - point).normalized();

        // 2. diffuse = kd * (l / r^2) * max(0, n·l)
        diffuse += kd.cwiseProduct(light.intensity / r_square) * std::max(0.0f, normal.dot(l));

        // 3. specular = ks * (l / r^2) * pow(max(0, n·h), p)
        // v 是着色点指向相机方向的向量

        Vector3f v = eye_pos - point;
        // h 是 l 和 v 的对角线单位向量
        Vector3f h = (l + v).normalized();
        specular += ks.cwiseProduct(light.intensity / r_square) * pow(std::max(0.0f, normal.dot(h)), p);
    }
    result_color = ambient + diffuse + specular;
    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    // 1. 题目给的方式
    // auto[x, y, z] = std::tuple{normal[0], normal[1], normal[2]};
    // Vector3f t = {x*y/sqrt(x*x+z*z), sqrt(x*x+z*z), z*y/sqrt(x*x+z*z)};
    // Vector3f b = normal.cross(t);
    // // tangent、bitangent、normal
    // Matrix3f TBN;
    // TBN << t, b, normal;

    // float u = payload.tex_coords.x();
    // float v = payload.tex_coords.y();
    // float w = payload.texture->width;
    // float h = payload.texture->height;
    // float dU = kh * kn * (payload.texture->getColor(u + 1.0f/w, v).norm() - payload.texture->getColor(u, v).norm());
    // float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f/h).norm() - payload.texture->getColor(u, v).norm());

    // // 其实就是 uv 方向的切线叉乘 => (1, 0, dU) 叉乘 (0, 1, dV)
    // Eigen::Vector3f ln = Eigen::Vector3f{-dU, -dV, 1.0f};
    // normal = TBN * ln;
    // Eigen::Vector3f result_color = normal.normalized();

    // 2. 另一种方式求切线
    Vector3f t = (payload.T).normalized();
    Vector3f b = normal.cross(t).normalized();
    Matrix3f TBN{
        {t.x(), b.x(), normal.x()},
        {t.y(), b.z(), normal.y()},
        {t.z(), b.z(), normal.z()}
    };

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    // kh kn 表示纹理定义的法线对真实物体法线的影响系数（程度）
    // 然后这里注意了，因为这个 Demo 问题，这个名称如果不清楚会很容易误解
    // 我们测试 凹凸贴图 的时候，给 texture 传入的是 hmap（高度图），而不是我们惯性思维上的普通纹理贴图

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f/h).norm() - payload.texture->getColor(u, v).norm());

    // 将这个坐标写成(s,v,u)移动在p点移动1个单位s，u的变化是dp/du,v的变化是dp/dv，然后得到一个点，
    // 这个点减去p点（这里是视频中的着色点）就是一个p到该点的向量，然后叉乘这个向量得到法向量.
    // u v切线的方向向量的叉乘就是法线方向
    // 其实就是 uv 方向的切线叉乘 => (1, 0, dU) 叉乘 (0, 1, dV)
    Eigen::Vector3f ln = Eigen::Vector3f{-dU, -dV, 1.0f};
    normal = TBN * ln;

    // 凹凸贴图
    Eigen::Vector3f result_color = normal.normalized();

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "E:/Dev/GAMES101/Homework3/Assignment3/models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("E:/Dev/GAMES101/Homework3/Assignment3/models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    // auto texture_path = "hmap.jpg";
    auto texture_path = "spot_texture.png";
    r.set_texture(Texture(obj_path + texture_path));

    // std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = normal_fragment_shader;
    // std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
    // std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = bump_fragment_shader;
    // std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = displacement_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader/n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader/n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader/n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader/n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader/n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
