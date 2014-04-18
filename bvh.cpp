#include <accel/BVH.h>
#include <accel/SplitBVHBuilder.h>

BVH::BVH(Scene* scene, const Platform& platform, const BuildParams& params)
{
    assert(scene);
    m_scene = scene;
    m_platform = platform;

    if (params.enablePrints)
		printf("BVH builder: %d tris, %d vertices\n", scene->getNumTriangles(), scene->getNumVertices());

    m_root = SplitBVHBuilder(*this, params).run();

	AABB scene_bound = scene->getSceneAabb();
    if (params.enablePrints)
		printf("BVH: Scene bounds: (%f,%f,%f) - (%f,%f,%f)\n", scene_bound.bmin.x, scene_bound.bmin.y, scene_bound.bmin.z,
                                                                           scene_bound.bmax.x, scene_bound.bmax.y, scene_bound.bmax.z);

    float sah = 0.f;
    m_root->computeSubtreeProbabilities(m_platform, 1.f, sah);
    if (params.enablePrints)
        printf("top-down sah: %.2f\n", sah);

    if(params.stats)
    {
        params.stats->SAHCost           = sah;
        params.stats->branchingFactor   = 2;
        params.stats->numLeafNodes      = m_root->getSubtreeSize(BVH_STAT_LEAF_COUNT);
        params.stats->numInnerNodes     = m_root->getSubtreeSize(BVH_STAT_INNER_COUNT);
        params.stats->numTris           = m_root->getSubtreeSize(BVH_STAT_TRIANGLE_COUNT);
        params.stats->numChildNodes     = m_root->getSubtreeSize(BVH_STAT_CHILDNODE_COUNT);
    }
}

static int currentTreelet;

void BVH::trace(std::vector<Ray>& rays, std::vector<RayResult>& results, bool needClosestHit, RayStats* stats) const
{
	for(size_t i=0;i<rays.size();i++)
    {
        Ray ray = rays[i];    // takes a local copy
        RayResult& result = results[i];

        result.clear();

        currentTreelet = -2;
        
		if(stats)
        {
            stats->platform = m_platform;
            stats->numRays++;
        }

        traceRecursive(m_root, ray, result, needClosestHit, stats);
    }
}

void BVH::traceRecursive(BVHNode* node, Ray& ray, RayResult& result,bool needClosestHit, RayStats* stats) const
{
    if(currentTreelet != node->m_treelet)
    {
        if(stats)
        {
//          if(!uniqueTreelets.contains(node->m_treelet))   // count unique treelets (comment this line to count all)
                stats->numTreelets++;
        }
        currentTreelet = node->m_treelet;
    }

    if(node->isLeaf())
    {
        const LeafNode* leaf        = reinterpret_cast<const LeafNode*>(node);
        const int3*     triVtxIndex = (const int3*)m_scene->getTriVtxIndexBuffer();
        const float3*   vtxPos      = (const float3*)m_scene->getVtxPosBuffer();

        if(stats)
            stats->numTriangleTests += m_platform.roundToTriangleBatchSize( leaf->getNumTriangles() );

        for(int i=leaf->m_lo; i<leaf->m_hi; i++)
        {
            int index = m_triIndices[i];
            const int3& ind = triVtxIndex[index];
            const float3& v0 = vtxPos[ind.x];
            const float3& v1 = vtxPos[ind.y];
            const float3& v2 = vtxPos[ind.z];
			float3 bary;
			bool hit = RayTriangle(v0, v1, v2, ray, bary);
            //Vec3f bary = Intersect::RayTriangle(v0,v1,v2, ray);
            float t = bary.z;

            if(t>ray.tmin && t<ray.tmax && hit)
            {
                ray.tmax    = t;
                result.t    = t;
                result.id   = index;

                if(!needClosestHit)
                    return;
            }
        }
    }
    else
    {
        if(stats)
            stats->numNodeTests += m_platform.roundToNodeBatchSize( node->getNumChildNodes() );

        const int TMIN = 0;
        const int TMAX = 1;
		float2    tspan0, tspan1;
        const InnerNode* inner = reinterpret_cast<const InnerNode*>(node);
        BVHNode* child0 = inner->m_children[0];
        BVHNode* child1 = inner->m_children[1];
		bool intersect0 = RayBox(child0->m_bounds, ray, tspan0);
		bool intersect1 = RayBox(child1->m_bounds, ray, tspan1);

        if(intersect0 && intersect1)
        if(tspan0.x > tspan1.x)
        {
            std::swap(tspan0,tspan1);
            std::swap(child0,child1);
        }

        if(intersect0)
            traceRecursive(child0,ray,result,needClosestHit,stats);

        if(result.hit() && !needClosestHit)
            return;

//      if(tspan1[TMIN] <= ray.tmax)    // this test helps only about 1-2%
        if(intersect1)
            traceRecursive(child1, ray, result, needClosestHit, stats);
    }
}
