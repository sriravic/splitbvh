#ifndef __BVH_H__
#define __BVH_H__

#include <accel/BVHNode.h>
#include <accel/Platform.h>
#include <core/primitives.h>
#include <core/scene.h>


struct RayStats
{
    RayStats()          { clear(); }
    void clear()        { memset(this,0,sizeof(RayStats)); }
    void print() const  { 
		if(numRays>0) 
			printf("Ray stats: (%s) %d rays, %.1f tris/ray, %.1f nodes/ray (cost=%.2f) %.2f treelets/ray\n", 
			platform.getName().c_str(), 
			numRays, 
			1.f*numTriangleTests/numRays, 
			1.f*numNodeTests/numRays, 
			(platform.getSAHTriangleCost()*numTriangleTests/numRays + platform.getSAHNodeCost()*numNodeTests/numRays), 
			1.f*numTreelets/numRays ); 
	}

    int         numRays;
    int         numTriangleTests;
    int         numNodeTests;
    int         numTreelets;
    Platform    platform;           // set by whoever sets the stats
};

class BVH
{
public:
    struct Stats
    {
        Stats()             { clear(); }
        void clear()        { memset(this, 0, sizeof(Stats)); }
        void print() const  { 
			printf("Tree stats: [bfactor=%d] %d nodes (%d+%d), %.2f SAHCost, %.1f children/inner, %.1f tris/leaf\n", 
				branchingFactor,
				numLeafNodes+numInnerNodes, 
				numLeafNodes,numInnerNodes, 
				SAHCost, 
				1.f*numChildNodes/std::max(numInnerNodes, 1), 
				1.f*numTris/std::max(numLeafNodes, 1)); 
		}

        float     SAHCost;
        int     branchingFactor;
        int     numInnerNodes;
        int     numLeafNodes;
        int     numChildNodes;
        int     numTris;
    };

    struct BuildParams
    {
        Stats*      stats;
        bool        enablePrints;
        float         splitAlpha;     // spatial split area threshold

        BuildParams(void)
        {
            stats           = NULL;
            enablePrints    = true;
            splitAlpha      = 1.0e-5f;
        }
    };

public:
                        BVH                     (Scene* scene, const Platform& platform, const BuildParams& params);
                        ~BVH                    (void)                  { if(m_root) m_root->deleteSubtree(); }

    Scene*              getScene                (void) const            { return m_scene; }
    const Platform&     getPlatform             (void) const            { return m_platform; }
    BVHNode*            getRoot                 (void) const            { return m_root; }
    void                trace                   (std::vector<Ray>& rays, std::vector<RayResult>& results, bool needClosestHit, RayStats* stats = NULL) const;

    std::vector<int>&         getTriIndices           (void)                  { return m_triIndices; }
    const std::vector<int>&   getTriIndices           (void) const            { return m_triIndices; }

private:
    void                traceRecursive          (BVHNode* node, Ray& ray, RayResult& result, bool needClosestHit, RayStats* stats) const;

    Scene*              m_scene;
    Platform            m_platform;

    BVHNode*            m_root;
    std::vector<int>    m_triIndices;
};


#endif
