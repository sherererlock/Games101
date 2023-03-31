```mermaid
classDiagram
class Object{
	+bool intersect(const Ray& ray);
	+Intersetion getInterSection(Rat ray);
	+Bounds3 getBound();
}

class Triangle{
	Vector3f v0, v1, v2;
	Vector3f t0, t1, t2;
	Vector3f normal;
}

Object <|-- Triangle

class MeshTriangle{
	Bounds3 bounding_box;
	Vector3f[] vertices;
	uint32_t numTriangles;
	
    BVHAccel* bvh;
    Material* m;
    std::vector~Triangle~ triangles;
}

Object <|-- MeshTriangle 

class Scene{
	int width;
	int height;
	BVHAccel *bvh;
	std::vector~Object*~ objects;
	
	void Add(Object *object);
	Intersection intersect(const Ray& ray) const;
	+ void buildBVH();
	+ Vector3f castRay(const Ray& ray, int depth);
	+ bool trace(Ray& ray,vector~Object*~& objects, float& tnear, int index, Object** hitObject);
}

Scene o-- Object

class BVHAccel {
	BVHBuildNode* root;
	std::vector~Object*~ primitives;
	
	BVHBuildNode* recursiveBuild(std::vector~Object*~objects);
	bool IntersectP(const Ray &ray) const;
}

class BVHBuildNode{
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;
    int nPrimitives;
}

BVHAccel *-- BVHBuildNode

Scene *-- BVHAccel
MeshTriangle *-- BVHAccel
```

```c++
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    // 创建返回的BVHBuildNode
    
    // 求出objects的boundingbox
    
    // 只有一个object
    // bounds等于object的Bounds
    
    // 有两个object
    // 用两个object分别构建left和right node，递归调用recursiveBuild
    // bounds等于左右两个子树bounds的Union
    
    // 有三个或以上object
    // 找出最长的轴，并将所有object用最长轴的坐标大小排序
    // 从中点一分为二递归调用recursiveBuild,分别构建左右子树
    // bounds等于左右两个子树bounds的Union
}

int main()
{
    MeshTriangle();
    {
        // 加载模型
        
        //每三个顶点组成一个三角形，添加到Objects数组中，并用这个数组构建BVH树
        
        //计算整个模型的boundingbox
    }
    
   	// 将MeshTriangle添加到scene的object数组中，且用此数组构建bvh
    // 
}

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection intersection = Scene::intersect(ray);
    {
        //this->bvh->Intersect(ray); 展开后
        Intersection BVHAccel::Intersect(const Ray& ray) const
        {
            Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
            {
                
            }
        }
    }
}
```

