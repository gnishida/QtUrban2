#pragma once

#include "RoadGraph.h"

class ShapeDetector {
public:
	ShapeDetector() {}
	~ShapeDetector() {}

	static std::vector<RoadEdgeDescs> detect(RoadGraph &roads, float scale, float threshold);
	static RoadEdgeDescs addVerticesToCircle(RoadGraph &roads, RoadEdgeDescs& shape, float threshold, QMap<RoadVertexDesc, bool> &usedVertices, QMap<RoadEdgeDesc, int> &usedEdges);
	static void addVerticesToGroup(RoadGraph &roads, RoadVertexDesc v, float threshold, RoadEdgeDescs &shape, QMap<RoadVertexDesc, bool> &usedVertices, QMap<RoadEdgeDesc, int> &usedEdges);
	static void savePatchImages(RoadGraph& roads, std::vector<RoadEdgeDescs> shapes);
};
