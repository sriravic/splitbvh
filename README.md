splitbvh
========

CPU SplitBVH Builder
********************

The SplitBVH builder is code that is modified from the original source code provided by NVIDIA in their ray traversal kernel paper. It has been modified not to use overly complicated nvidia code that. 

The original code had a lot of 'fat' associated with it. It had a complete framework custom written for their demo. I decided to modify the code so that I can better use it with applications instead of using their entire framework just for the functionality the bvh builder provides. 

This code uses simple std::vectors instead of their custom buffers. Further I've removed most of the code that had windows dependencies so that its easier to use in any platform with STL. Note that the code does depend on some basic form of 3-vector processing and AABB structures. I have used CUDA float3 and my own AABB implementation which I assume the user to also use or have. If not, its easier to write your own. It need not have complex functionalities.

The bvh tracer is CPU based. I will try to modify the cuda version and get that also posted as a separate project. 

NOTE: Its a research record that no acceleration structure can beat the performance of the split bvh. Its the gold standard. Hence I guess people who want to test their acceleration structures should use this code to check their performance. 
