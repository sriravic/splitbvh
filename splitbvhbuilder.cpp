#include <accel/SplitBVHBuilder.h>

SplitBVHBuilder::SplitBVHBuilder(BVH& bvh, const BVH::BuildParams& params)
:   m_bvh           (bvh),
    m_platform      (bvh.getPlatform()),
    m_params        (params),
    m_minOverlap    (0.0f),
    m_sortDim       (-1)
{
	// create the timer
	if(!sdkCreateTimer(&m_progressTimer)) {
		printf("Error creating Progress Timer for SplitBVHBuilder\n");
	}
}

//------------------------------------------------------------------------

SplitBVHBuilder::~SplitBVHBuilder(void)
{
	sdkDeleteTimer(&m_progressTimer);
}

BVHNode* SplitBVHBuilder::run(void)
{
    // Initialize reference stack and determine root bounds.

    const int3* tris    = (const int3*)m_bvh.getScene()->getTriVtxIndexBuffer();
    const float3* verts = (const float3*)m_bvh.getScene()->getVtxPosBuffer();

    NodeSpec rootSpec;
    rootSpec.numRef = m_bvh.getScene()->getNumTriangles();
    m_refStack.resize(rootSpec.numRef);

    for (int i = 0; i < rootSpec.numRef; i++)
    {
        m_refStack[i].triIdx = i;
		/*
        for (int j = 0; j < 3; j++)
            m_refStack[i].bounds.grow(verts[tris[i][j]]);
		*/
		// we unroll the above logic because we dont have a [] operator for the float3/int3 types
		m_refStack[i].bounds.grow(verts[tris[i].x]);
		m_refStack[i].bounds.grow(verts[tris[i].y]);
		m_refStack[i].bounds.grow(verts[tris[i].z]);

        rootSpec.bounds.grow(m_refStack[i].bounds);
    }

	// we can do a sanity check to see if the rootspec bounds is the same is the scene bounds.
    
	// Initialize rest of the members.
	m_minOverlap = rootSpec.bounds.area() * m_params.splitAlpha;
    //m_rightBounds.reset(max(rootSpec.numRef, (int)NumSpatialBins) - 1);
	m_rightBounds.resize(max(rootSpec.numRef, (int)NumSpatialBins) - 1);
    m_numDuplicates = 0;
    m_progressTimer->start();

    // Build recursively.

    BVHNode* root = buildNode(rootSpec, 0, 0.0f, 1.0f);
    //m_bvh.getTriIndices().compact();
	//m_bvh.getTriIndices().clear();

    // Done.

    if (m_params.enablePrints)
        printf("SplitBVHBuilder: progress %.0f%%, duplicates %.0f%%\n",
            100.0f, (float)m_numDuplicates / (float)m_bvh.getScene()->getNumTriangles() * 100.0f);
    return root;
}

bool SplitBVHBuilder::sortCompare(void* data, int idxA, int idxB)
{
    const SplitBVHBuilder* ptr = (const SplitBVHBuilder*)data;
    int dim = ptr->m_sortDim;
    const Reference& ra = ptr->m_refStack[idxA];
    const Reference& rb = ptr->m_refStack[idxB];
    float ca = RenderBox::get(ra.bounds.bmin, dim) + RenderBox::get(ra.bounds.bmax, dim);
    float cb = RenderBox::get(rb.bounds.bmin, dim) + RenderBox::get(rb.bounds.bmax, dim);
    return (ca < cb || (ca == cb && ra.triIdx < rb.triIdx));
}

void SplitBVHBuilder::sortSwap(void* data, int idxA, int idxB)
{
    SplitBVHBuilder* ptr = (SplitBVHBuilder*)data;
    std::swap(ptr->m_refStack[idxA], ptr->m_refStack[idxB]);
}

BVHNode* SplitBVHBuilder::buildNode(NodeSpec spec, int level, float progressStart, float progressEnd)
{
    // Display progress.

	if (m_params.enablePrints && m_progressTimer->getTime() >= 1000.0f)
    {
        printf("SplitBVHBuilder: progress %.0f%%, duplicates %.0f%%\r",
            progressStart * 100.0f, (float)m_numDuplicates / (float)m_bvh.getScene()->getNumTriangles() * 100.0f);
		m_progressTimer->start();
    }

    // Remove degenerates.
    {
        int firstRef = m_refStack.size() - spec.numRef;
        for (int i = m_refStack.size() - 1; i >= firstRef; i--)
        {
            float3 size = m_refStack[i].bounds.bmax - m_refStack[i].bounds.bmin;
			if (RenderBox::getMin(size) < 0.0f || RenderBox::getSum(size) == RenderBox::getMax(size)) {
#ifdef _DEBUG
				printf("Degenerate triangle found\n");
#endif
				removeSwap(m_refStack, i);
			}
                //m_refStack.removeSwap(i);
        }
        spec.numRef = m_refStack.size() - firstRef;
    }

    // Small enough or too deep => create leaf.

    if (spec.numRef <= m_platform.getMinLeafSize() || level >= MaxDepth)
        return createLeaf(spec);

    // Find split candidates.

    float area = spec.bounds.area();
    float leafSAH = area * m_platform.getTriangleCost(spec.numRef);
    float nodeSAH = area * m_platform.getNodeCost(2);
    ObjectSplit object = findObjectSplit(spec, nodeSAH);

	
    SpatialSplit spatial;
    if (level < MaxSpatialDepth)
    {
        AABB overlap = object.leftBounds;
        overlap.intersect(object.rightBounds);
        if (overlap.area() >= m_minOverlap)
            spatial = findSpatialSplit(spec, nodeSAH);
    }

    // Leaf SAH is the lowest => create leaf.

    float minSAH = std::min(leafSAH, std::min(object.sah, spatial.sah));
	//float minSAH = std::min(leafSAH, object.sah);
    if (minSAH == leafSAH && spec.numRef <= m_platform.getMaxLeafSize())
        return createLeaf(spec);

    // Perform split.

    NodeSpec left, right;
    if (minSAH == spatial.sah)
        performSpatialSplit(left, right, spec, spatial);
    if (!left.numRef || !right.numRef)
        performObjectSplit(left, right, spec, object);

    // Create inner node.

    m_numDuplicates += left.numRef + right.numRef - spec.numRef;
    float progressMid = lerp(progressStart, progressEnd, (float)right.numRef / (float)(left.numRef + right.numRef));
    BVHNode* rightNode = buildNode(right, level + 1, progressStart, progressMid);
    BVHNode* leftNode = buildNode(left, level + 1, progressMid, progressEnd);
    return new InnerNode(spec.bounds, leftNode, rightNode);
}

BVHNode* SplitBVHBuilder::createLeaf(const NodeSpec& spec)
{
    std::vector<int>& tris = m_bvh.getTriIndices();
    for (int i = 0; i < spec.numRef; i++) {
		Reference last = removeLast(m_refStack);
		tris.push_back(last.triIdx);
        //tris.push_back(m_refStack.removeLast().triIdx);
	}
    return new LeafNode(spec.bounds, tris.size() - spec.numRef, tris.size());
}

SplitBVHBuilder::ObjectSplit SplitBVHBuilder::findObjectSplit(const NodeSpec& spec, float nodeSAH)
{
    ObjectSplit split;
    //const Reference* refPtr = m_refStack.getPtr(m_refStack.getSize() - spec.numRef);
	const Reference* refPtr = (const Reference*)&(m_refStack[m_refStack.size() - spec.numRef]);
    float bestTieBreak = FLT_MAX;

    // Sort along each dimension.

    for (m_sortDim = 0; m_sortDim < 3; m_sortDim++)
    {
        sort(this, m_refStack.size() - spec.numRef, m_refStack.size(), sortCompare, sortSwap);

        // Sweep right to left and determine bounds.

        AABB rightBounds;
        for (int i = spec.numRef - 1; i > 0; i--)
        {
            rightBounds.grow(refPtr[i].bounds);
            m_rightBounds[i - 1] = rightBounds;
        }

        // Sweep left to right and select lowest SAH.

        AABB leftBounds;
        for (int i = 1; i < spec.numRef; i++)
        {
            leftBounds.grow(refPtr[i - 1].bounds);
            float sah = nodeSAH + leftBounds.area() * m_platform.getTriangleCost(i) + m_rightBounds[i - 1].area() * m_platform.getTriangleCost(spec.numRef - i);
            float tieBreak = RenderBox::sqr((float)i) + RenderBox::sqr((float)(spec.numRef - i));
            if (sah < split.sah || (sah == split.sah && tieBreak < bestTieBreak))
            {
                split.sah = sah;
                split.sortDim = m_sortDim;
                split.numLeft = i;
                split.leftBounds = leftBounds;
                split.rightBounds = m_rightBounds[i - 1];
                bestTieBreak = tieBreak;
            }
        }
    }
    return split;
}
void SplitBVHBuilder::performObjectSplit(NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const ObjectSplit& split)
{
    m_sortDim = split.sortDim;
    sort(this, m_refStack.size() - spec.numRef, m_refStack.size(), sortCompare, sortSwap);

    left.numRef = split.numLeft;
    left.bounds = split.leftBounds;
    right.numRef = spec.numRef - split.numLeft;
    right.bounds = split.rightBounds;
}

SplitBVHBuilder::SpatialSplit SplitBVHBuilder::findSpatialSplit(const NodeSpec& spec, float nodeSAH)
{
    // Initialize bins.

    float3 origin = spec.bounds.bmin;
    float3 binSize = (spec.bounds.bmax - origin) * (1.0f / (float)NumSpatialBins);
    float3 invBinSize = 1.0f / binSize;

    for (int dim = 0; dim < 3; dim++)
    {
        for (int i = 0; i < NumSpatialBins; i++)
        {
            SpatialBin& bin = m_bins[dim][i];
            bin.bounds = AABB();
            bin.enter = 0;
            bin.exit = 0;
        }
    }

    // Chop references into bins.

    for (size_t refIdx = m_refStack.size() - spec.numRef; refIdx < m_refStack.size(); refIdx++)
    {
        const Reference& ref = m_refStack[refIdx];
		
		float3 refmin = (ref.bounds.bmin - origin) * invBinSize;
		float3 refmax = (ref.bounds.bmax - origin) * invBinSize;
        int3 firstBin = clamp(make_int3((int)refmin.x, (int)refmin.y, (int)refmin.z), 0, NumSpatialBins - 1);
        int3 lastBin  = clamp(make_int3((int)refmax.x, (int)refmax.y, (int)refmax.z), firstBin, make_int3(NumSpatialBins - 1));

        for (int dim = 0; dim < 3; dim++)
        {
            Reference currRef = ref;
            for (int i = RenderBox::get(firstBin, dim); i < RenderBox::get(lastBin, dim); i++)
            {
                Reference leftRef, rightRef;
                splitReference(leftRef, rightRef, currRef, dim, RenderBox::get(origin, dim) + RenderBox::get(binSize, dim) * (float)(i + 1));
                m_bins[dim][i].bounds.grow(leftRef.bounds);
                currRef = rightRef;
            }
            m_bins[dim][RenderBox::get(lastBin, dim)].bounds.grow(currRef.bounds);
            m_bins[dim][RenderBox::get(firstBin, dim)].enter++;
            m_bins[dim][RenderBox::get(lastBin, dim)].exit++;
        }
    }

    // Select best split plane.

    SpatialSplit split;
    for (int dim = 0; dim < 3; dim++)
    {
        // Sweep right to left and determine bounds.

        AABB rightBounds;
        for (int i = NumSpatialBins - 1; i > 0; i--)
        {
            rightBounds.grow(m_bins[dim][i].bounds);
            m_rightBounds[i - 1] = rightBounds;
        }

        // Sweep left to right and select lowest SAH.

        AABB leftBounds;
        int leftNum = 0;
        int rightNum = spec.numRef;

        for (int i = 1; i < NumSpatialBins; i++)
        {
            leftBounds.grow(m_bins[dim][i - 1].bounds);
            leftNum += m_bins[dim][i - 1].enter;
            rightNum -= m_bins[dim][i - 1].exit;

            float sah = nodeSAH + leftBounds.area() * m_platform.getTriangleCost(leftNum) + m_rightBounds[i - 1].area() * m_platform.getTriangleCost(rightNum);
            if (sah < split.sah)
            {
                split.sah = sah;
                split.dim = dim;
                split.pos = RenderBox::get(origin, dim) + RenderBox::get(binSize, dim) * (float)i;
            }
        }
    }
    return split;
}

void SplitBVHBuilder::performSpatialSplit(NodeSpec& left, NodeSpec& right, const NodeSpec& spec, const SpatialSplit& split)
{
    // Categorize references and compute bounds.
    //
    // Left-hand side:      [leftStart, leftEnd[
    // Uncategorized/split: [leftEnd, rightStart[
    // Right-hand side:     [rightStart, refs.getSize()[

    std::vector<Reference>& refs = m_refStack;
    int leftStart = refs.size() - spec.numRef;
    int leftEnd = leftStart;
    int rightStart = refs.size();
    left.bounds = right.bounds = AABB();

    for (int i = leftEnd; i < rightStart; i++)
    {
        // Entirely on the left-hand side?

        if (RenderBox::get(refs[i].bounds.bmax, split.dim) <= split.pos)
        {
            left.bounds.grow(refs[i].bounds);
            std::swap(refs[i], refs[leftEnd++]);
        }

        // Entirely on the right-hand side?

        else if (RenderBox::get(refs[i].bounds.bmin, split.dim) >= split.pos)
        {
            right.bounds.grow(refs[i].bounds);
            std::swap(refs[i--], refs[--rightStart]);
        }
    }

    // Duplicate or unsplit references intersecting both sides.

    while (leftEnd < rightStart)
    {
        // Split reference.

        Reference lref, rref;
        splitReference(lref, rref, refs[leftEnd], split.dim, split.pos);

        // Compute SAH for duplicate/unsplit candidates.

        AABB lub = left.bounds;  // Unsplit to left:     new left-hand bounds.
        AABB rub = right.bounds; // Unsplit to right:    new right-hand bounds.
        AABB ldb = left.bounds;  // Duplicate:           new left-hand bounds.
        AABB rdb = right.bounds; // Duplicate:           new right-hand bounds.
        lub.grow(refs[leftEnd].bounds);
        rub.grow(refs[leftEnd].bounds);
        ldb.grow(lref.bounds);
        rdb.grow(rref.bounds);

        float lac = m_platform.getTriangleCost(leftEnd - leftStart);
        float rac = m_platform.getTriangleCost(refs.size() - rightStart);
        float lbc = m_platform.getTriangleCost(leftEnd - leftStart + 1);
        float rbc = m_platform.getTriangleCost(refs.size() - rightStart + 1);

        float unsplitLeftSAH = lub.area() * lbc + right.bounds.area() * rac;
        float unsplitRightSAH = left.bounds.area() * lac + rub.area() * rbc;
        float duplicateSAH = ldb.area() * lbc + rdb.area() * rbc;
        float minSAH = std::min(unsplitLeftSAH, std::min(unsplitRightSAH, duplicateSAH));

        // Unsplit to left?

        if (minSAH == unsplitLeftSAH)
        {
            left.bounds = lub;
            leftEnd++;
        }

        // Unsplit to right?

        else if (minSAH == unsplitRightSAH)
        {
            right.bounds = rub;
            std::swap(refs[leftEnd], refs[--rightStart]);
        }

        // Duplicate?

        else
        {
            left.bounds = ldb;
            right.bounds = rdb;
            refs[leftEnd++] = lref;
            //refs.add(rref);
			refs.push_back(rref);
        }
    }

    left.numRef = leftEnd - leftStart;
    right.numRef = refs.size() - rightStart;
}

void SplitBVHBuilder::splitReference(Reference& left, Reference& right, const Reference& ref, int dim, float pos)
{
    // Initialize references.

    left.triIdx = right.triIdx = ref.triIdx;
    left.bounds = right.bounds = AABB();

    // Loop over vertices/edges.

    const int3* tris = (const int3*)m_bvh.getScene()->getTriVtxIndexBuffer();
    const float3* verts = (const float3*)m_bvh.getScene()->getVtxPosBuffer();
    const int3& inds = tris[ref.triIdx];
    float3 v1 = verts[inds.z];

    for (int i = 0; i < 3; i++)
    {
        float3 v0 = v1;
		//int id = RenderBox::get<int3, int, 3>(inds, i);
		int id = RenderBox::get(inds, i);
        v1 = verts[id];
        float v0p = RenderBox::get(v0, dim);
        float v1p = RenderBox::get(v1, dim);

        // Insert vertex to the boxes it belongs to.

        if (v0p <= pos)
            left.bounds.grow(v0);
        if (v0p >= pos)
            right.bounds.grow(v0);

        // Edge intersects the plane => insert intersection to both boxes.

        if ((v0p < pos && v1p > pos) || (v0p > pos && v1p < pos))
        {
            float3 t = lerp(v0, v1, clamp((pos - v0p) / (v1p - v0p), 0.0f, 1.0f));
            left.bounds.grow(t);
            right.bounds.grow(t);
        }
    }

    // Intersect with original bounds.

    RenderBox::get(left.bounds.bmax, dim) = pos;
    RenderBox::get(right.bounds.bmin, dim) = pos;
    left.bounds.intersect(ref.bounds);
    right.bounds.intersect(ref.bounds);
}
