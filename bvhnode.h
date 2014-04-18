#ifndef __BVH_NODE_H__
#define __BVH_NODE_H__

#include <accel/Platform.h>
#include <core/primitives.h>

enum BVH_STAT
{
    BVH_STAT_NODE_COUNT,
    BVH_STAT_INNER_COUNT,
    BVH_STAT_LEAF_COUNT,
    BVH_STAT_TRIANGLE_COUNT,
    BVH_STAT_CHILDNODE_COUNT,
};

class BVHNode
{
public:
    BVHNode() : m_probability(1.f),m_parentProbability(1.f),m_treelet(-1),m_index(-1) {}
    virtual bool        isLeaf() const = 0;
    virtual int         getNumChildNodes() const = 0;
    virtual BVHNode*    getChildNode(int i) const   = 0;
    virtual int         getNumTriangles() const { return 0; }

    float       getArea() const     { return m_bounds.area(); }

    AABB        m_bounds;

    // These are somewhat experimental, for some specific test and may be invalid...
    float       m_probability;          // probability of coming here (widebvh uses this)
    float       m_parentProbability;    // probability of coming to parent (widebvh uses this)

    int         m_treelet;              // for queuing tests (qmachine uses this)
    int         m_index;                // in linearized tree (qmachine uses this)

    // Subtree functions
    int     getSubtreeSize(BVH_STAT stat=BVH_STAT_NODE_COUNT) const;
    void    computeSubtreeProbabilities(const Platform& p, float parentProbability, float& sah);
    float   computeSubtreeSAHCost(const Platform& p) const;     // NOTE: assumes valid probabilities
    void    deleteSubtree();

    void    assignIndicesDepthFirst  (int index=0, bool includeLeafNodes=true);
    void    assignIndicesBreadthFirst(int index=0, bool includeLeafNodes=true);
};


class InnerNode : public BVHNode
{
public:
    InnerNode(const AABB& bounds,BVHNode* child0,BVHNode* child1)   { m_bounds=bounds; m_children[0]=child0; m_children[1]=child1; }

    bool        isLeaf() const                  { return false; }
    int         getNumChildNodes() const        { return 2; }
    BVHNode*    getChildNode(int i) const       { assert(i>=0 && i<2); return m_children[i]; }

    BVHNode*    m_children[2];
};


class LeafNode : public BVHNode
{
public:
    LeafNode(const AABB& bounds,int lo,int hi)  { m_bounds=bounds; m_lo=lo; m_hi=hi; }
    LeafNode(const LeafNode& s)                 { *this = s; }

    bool        isLeaf() const                  { return true; }
    int         getNumChildNodes() const        { return 0; }
    BVHNode*    getChildNode(int) const         { return NULL; }

    int         getNumTriangles() const         { return m_hi-m_lo; }
    int         m_lo;
    int         m_hi;
};


#endif
