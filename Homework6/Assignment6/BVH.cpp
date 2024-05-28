#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        bool SAH = true;

        if (SAH)
        {
            int part = 10;
            auto size = objects.size();
            int proper_cut = 0;
            double mintime = DBL_MAX;
            for (int index = 0; index < part; index++)
            {
                middling = objects.begin() + size * index / part;
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                Bounds3 leftBounds, rightBounds;
                for (int i = 0; i < leftshapes.size(); i++)
                {
                    leftBounds = Union(leftBounds, leftshapes[i]->getBounds().Centroid());
                }
                for (int i = 0; i < rightshapes.size(); i++)
                {
                    rightBounds = Union(rightBounds, rightshapes[i]->getBounds().Centroid());
                }

                auto leftS = leftBounds.SurfaceArea();
                auto rightS = rightBounds.SurfaceArea();
                auto S = leftS + rightS;

                //     time = S_1面积 /S_0面积 *S_1空间物体数 * t_obj    
                //              + S_2面积 /S_0面积 *S_2空间物体数 * t_obj

                auto time = leftS / S * leftshapes.size() + rightS / S * rightshapes.size();
                if (time < mintime)
                {
                    mintime = time;
                    proper_cut = index;
                }
            }
            middling = objects.begin() + size * proper_cut / part;
        }

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = (ray.direction[0]<0);
    dirIsNeg[1] = (ray.direction[1]<0);
    dirIsNeg[2] = (ray.direction[2]<0);

    Intersection inter;
    if (!node->bounds.IntersectP(ray, ray.direction, dirIsNeg))
        return inter;
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);

    Intersection l = getIntersection(node->left, ray);
    Intersection r = getIntersection(node->right, ray);
    return l.distance < r.distance ? l : r;
}