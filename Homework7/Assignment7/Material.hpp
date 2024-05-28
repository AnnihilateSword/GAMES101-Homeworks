//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, MICROFACET };

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    // 增加粗糙度和金属度

    float roughness;
    float metalness;

    inline Vector3f fresnelSchlick(float cosTheta, Vector3f F0)
    {
        return F0 + (Vector3f(1.0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
    }

    inline float DistributionGGX(Vector3f N, Vector3f H, float roughness)
    {
        float a      = roughness*roughness;
        float a2     = a*a;
        float NdotH  = std::max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH*NdotH;

        float num   = a2;
        float denom = (NdotH2 * (a2 - 1.0) + 1.0);
        denom = M_PI * denom * denom;

        return num / denom;
    }

    inline float GeometrySchlickGGX(float NdotV, float roughness)
    {
        float r = (roughness + 1.0);
        float k = (r*r) / 8.0;

        float num   = NdotV;
        float denom = NdotV * (1.0 - k) + k;

        return num / denom;
    }

    inline float GeometrySmith(float NdotV, float NdotL, float roughness)
    {
        float ggx2  = GeometrySchlickGGX(NdotV, roughness);
        float ggx1  = GeometrySchlickGGX(NdotL, roughness);

        return ggx1 * ggx2;
    }

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFACET:
        {
            // 随机一个 ε 和 φ

            float r0 = get_random_float();
            float r1 = get_random_float();
            float a2 = roughness * roughness;
            float phi = 2 * M_PI * r1;
            float theta = std::acos(sqrt((1 - r0) / (r0 * (a2 - 1) + 1)));

            // 单位向量半径就直接 1 了，转换为直角坐标系只需要用到 r*sinθ，所以这里直接乘上去了

            float r = std::sin(theta);
            return reflect(wi, toWorld(Vector3f(r * std::cos(phi), r * std::sin(phi), std::cos(theta)), N));

        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFACET:
        {
            Vector3f h = (wo - wi).normalized();
            float cosTheta = dotProduct(N, h);
            float a2 = roughness * roughness;
            float exp = (a2 - 1) * cosTheta * cosTheta + 1;
            float D = a2 / (M_PI * exp * exp);
            return D * cosTheta / (4.f * dotProduct(wo, h));
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model

            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            // cosθ 是入射光和法线的夹角，也就是光源方向和法线方向的夹角

            float cosTheta = dotProduct(N, wo);
            if(cosTheta > 0.f)
            {

                // 对于大多数电介质表面而言使用 0.04 作为基础反射率已经足够好了

                Vector3f F0(0.04f);
                Vector3f V = -wi;
                Vector3f L = wo;
                Vector3f H = (V + L).normalized();
                float NdotV = std::max(dotProduct(N, V), 0.f);
                float NdotL = cosTheta;

                // 直接光照情况下的 k 公式

                float k = (roughness + 1.f) * (roughness + 1.f) / 8.f;
                float D = DistributionGGX(N, H, roughness);
                float G = GeometrySmith(NdotV, NdotL, k);
                F0 = lerp(F0, Kd, metalness);
                Vector3f F = fresnelSchlick(dotProduct(H, V), F0);
                
                // float F;
                // fresnel(V, N, ior, F);

                Vector3f fs = D * G * F / (4.f * NdotV  * NdotL);

                // 菲涅尔项就是 ks， kd = 1-ks; (1 - F) 是因为这里除了镜面反射就是漫反射，F 表示镜面反射程度

                Vector3f fr = (Vector3f(1.f) - F) * Kd / M_PI;

                return fr + fs;
            }
            return Vector3f(0.f);
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
