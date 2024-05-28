//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"

using namespace Eigen;
class Triangle{

public:
    Vector4f v[3]; /*the original coordinates of the triangle, v0, v1, v2 in counter clockwise order*/
    /*Per vertex values*/
    Vector3f color[3]; //color at each vertex;
    Vector2f tex_coords[3]; //texture u,v
    Vector3f normal[3]; //normal vector for each vertex

    Texture *tex= nullptr;
    Triangle();

    Eigen::Vector4f a() const { return v[0]; }
    Eigen::Vector4f b() const { return v[1]; }
    Eigen::Vector4f c() const { return v[2]; }

    void setVertex(int ind, Vector4f ver); /*set i-th vertex coordinates */
    void setNormal(int ind, Vector3f n); /*set i-th vertex normal vector*/
    void setColor(int ind, float r, float g, float b); /*set i-th vertex color*/

    void setNormals(const std::array<Vector3f, 3>& normals);
    void setColors(const std::array<Vector3f, 3>& colors);
    void setTexCoord(int ind,Vector2f uv ); /*set i-th vertex texture coordinate*/
    std::array<Vector4f, 3> toVector4() const;

    // 计算切线
    // 参考：https://learnopengl-cn.github.io/05%20Advanced%20Lighting/04%20Normal%20Mapping/

    mutable Vector3f* T;

    void CalculateTVector() const
    {
        auto u1 = tex_coords[1][0] - tex_coords[0][0];
        auto u2 = tex_coords[2][0] - tex_coords[0][0];
        auto v1 = tex_coords[1][1] - tex_coords[0][1];
        auto v2 = tex_coords[2][1] - tex_coords[0][1];

        float ratio = 1.0f / (u1*v2-u2*v1);

        auto e1 = v[1] - v[0];
        auto e2 = v[2] - v[0];

        T = new Vector3f(ratio * (v2*e1.x() - v1*e2.x()),
                    ratio * (v2*e1.y() - v1*e2.y()),
                    ratio * (v2*e1.z() - v1*e2.z())
                );
    }
};






#endif //RASTERIZER_TRIANGLE_H
