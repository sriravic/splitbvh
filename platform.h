#ifndef __PLATFORM_H__
#define __PLATFORM_H__

class LeafNode;
class BVHNode;
class Platform
{
public:
    Platform()                                                                                                       { m_name=std::string("Default"); m_SAHNodeCost = 1.f; m_SAHTriangleCost = 1.f; m_nodeBatchSize = 1; m_triBatchSize = 1; m_minLeafSize=1; m_maxLeafSize=0x7FFFFFF; }
    Platform(const std::string& name,float nodeCost=1.f, float triCost=1.f, int nodeBatchSize=1, int triBatchSize=1) { m_name=name; m_SAHNodeCost = nodeCost; m_SAHTriangleCost = triCost; m_nodeBatchSize = nodeBatchSize; m_triBatchSize = triBatchSize; m_minLeafSize=1; m_maxLeafSize=0x7FFFFFF; }

    const std::string&   getName() const                { return m_name; }

    // SAH weights
    float getSAHTriangleCost() const                    { return m_SAHTriangleCost; }
    float getSAHNodeCost() const                        { return m_SAHNodeCost; }

    // SAH costs, raw and batched
    float getCost(int numChildNodes,int numTris) const  { return getNodeCost(numChildNodes) + getTriangleCost(numTris); }
    float getTriangleCost(int n) const                  { return roundToTriangleBatchSize(n) * m_SAHTriangleCost; }
    float getNodeCost(int n) const                      { return roundToNodeBatchSize(n) * m_SAHNodeCost; }

    // batch processing (how many ops at the price of one)
    int   getTriangleBatchSize() const                  { return m_triBatchSize; }
    int   getNodeBatchSize() const                      { return m_nodeBatchSize; }
    void  setTriangleBatchSize(int triBatchSize)        { m_triBatchSize = triBatchSize; }
    void  setNodeBatchSize(int nodeBatchSize)           { m_nodeBatchSize= nodeBatchSize; }
    int   roundToTriangleBatchSize(int n) const         { return ((n+m_triBatchSize-1)/m_triBatchSize)*m_triBatchSize; }
    int   roundToNodeBatchSize(int n) const             { return ((n+m_nodeBatchSize-1)/m_nodeBatchSize)*m_nodeBatchSize; }

    // leaf preferences
    void  setLeafPreferences(int minSize,int maxSize)   { m_minLeafSize=minSize; m_maxLeafSize=maxSize; }
    int   getMinLeafSize() const                        { return m_minLeafSize; }
    int   getMaxLeafSize() const                        { return m_maxLeafSize; }

private:
    std::string  m_name;
    float   m_SAHNodeCost;
    float   m_SAHTriangleCost;
    int     m_triBatchSize;
    int     m_nodeBatchSize;
    int     m_minLeafSize;
    int     m_maxLeafSize;
};


#endif
