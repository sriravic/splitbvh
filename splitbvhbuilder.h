#ifndef __SPLIT_BVH_BUILDER_H__
#define __SPLIT_BVH_BUILDER_H__

#include <accel/BVH.h>
#include <core/primitives.h>
#include <cuda_defines.h>
#include <util/sort.h>

class SplitBVHBuilder
{
private:
    enum
    {
        MaxDepth        = 64,
        MaxSpatialDepth = 48,
        NumSpatialBins  = 128,
    };

    struct Reference
    {
        int                 triIdx;
        AABB                bounds;

        Reference(void) : triIdx(-1) {}
    };

    struct NodeSpec
    {
        int                 numRef;
        AABB                bounds;

        NodeSpec(void) : numRef(0) {}
    };

    struct ObjectSplit
    {
        float               sah;
        int                 sortDim;
        int                 numLeft;
        AABB                leftBounds;
        AABB                rightBounds;

        ObjectSplit(void) : sah(FLT_MAX), sortDim(0), numLeft(0) {}
    };

    struct SpatialSplit
    {
        float               sah;
        int                 dim;
        float               pos;

        SpatialSplit(void) : sah(FLT_MAX), dim(0), pos(0.0f) {}
    };

    struct SpatialBin
    {
        AABB                bounds;
        int                 enter;
        int                 exit;
    };

public:
                            SplitBVHBuilder     (BVH& bvh, const BVH::BuildParams& params);
                            ~SplitBVHBuilder    (void);

    BVHNode*                run                 (void);

private:
    static bool             sortCompare         (void* data, int idxA, int idxB);
    static void             sortSwap            (void* data, int idxA, int idxB);

    BVHNode*                buildNode           (NodeSpec spec, int level, float progressStart, float progressEnd);
    BVHNode*                createLeaf          (const NodeSpec& spec);

    ObjectSplit             findObjectSplit     (const NodeSpec& spec, float nodeSAH);
    void                    performObjectSplit  (NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const ObjectSplit& split);

    SpatialSplit            findSpatialSplit    (const NodeSpec& spec, float nodeSAH);
    void                    performSpatialSplit (NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const SpatialSplit& split);
    void                    splitReference      (Reference& left, Reference& right, const Reference& ref, int dim, float pos);

private:
                            SplitBVHBuilder     (const SplitBVHBuilder&); // forbidden
    SplitBVHBuilder&        operator=           (const SplitBVHBuilder&); // forbidden

private:

	// some util functions added so that they are compatible with the original code provided in the NVIDIA sdk
	template<typename T>
	T removeSwap(std::vector<T>& V, size_t idx) {
		assert(idx >= 0 && idx < V.size());
		T old = V[idx];
		T back = V.back();
		V.pop_back();			// remove the last element which is basically an size-- operation
		if(idx < V.size())
			V[idx] = back;
		return old;
	}

	template<typename T>
	T removeLast(std::vector<T>& V) {
		T old = V.back();
		V.pop_back();
		return old;
	}

private:

    BVH&                    m_bvh;
    const Platform&         m_platform;
    const BVH::BuildParams& m_params;

    std::vector<Reference>  m_refStack;
    float                   m_minOverlap;
    std::vector<AABB>       m_rightBounds;
    int                     m_sortDim;
    SpatialBin              m_bins[3][NumSpatialBins];

    int                     m_numDuplicates;
};


#endif
