#include "ShapeDetector.h"
#include "GraphUtil.h"
#include <QHash>
#include "CircleHoughTransform.h"

std::vector<RoadEdgeDescs> ShapeDetector::detect(RoadGraph &roads, float scale, float threshold) {
	float threshold2 = SQR(threshold);

	RoadVertexDesc center;

	QMap<RoadVertexDesc, bool> usedVertices;
	QMap<RoadVertexDesc, bool> usedVerticesInCircles;
	QMap<RoadEdgeDesc, int> usedEdges;

	std::vector<RoadEdgeDescs> shapes;
	/*
	// detect circles
	shapes = CircleHoughTransform::detect(roads, scale);
	time_t start = clock();
	for (int i = 0; i < shapes.size(); ++i) {
		for (int j = 0; j < shapes[i].size(); ++j) {
			RoadVertexDesc src = boost::source(shapes[i][j], roads.graph);
			RoadVertexDesc tgt = boost::target(shapes[i][j], roads.graph);

			usedVertices[src] = true;
			usedVertices[tgt] = true;
			usedVerticesInCircles[src] = true;
			usedVerticesInCircles[tgt] = true;
			usedEdges[shapes[i][j]] = 2;
		}
	}
	time_t end = clock();
	std::cout << "Circle detection: " << (double)(end - start) / CLOCKS_PER_SEC << " [sec]" << std::endl;

	// expand circles
	for (int i = 0; i < shapes.size(); ++i) {
		QMap<RoadVertexDesc, bool> visited;
		for (int j = 0; j < shapes[i].size(); ++j) {
			RoadVertexDesc src = boost::source(shapes[i][j], roads.graph);
			RoadVertexDesc tgt = boost::target(shapes[i][j], roads.graph);
			if (!visited.contains(src)) {
				RoadEdgeDescs shape = addVerticesToCircle(roads, src, threshold, visited, usedEdges);
				for (int k = 0; k < shape.size(); ++k) {
					shapes[i].push_back(shape[k]);
				}
			}
			if (!visited.contains(tgt)) {
				RoadEdgeDescs shape = addVerticesToCircle(roads, tgt, threshold, visited, usedEdges);
				for (int k = 0; k < shape.size(); ++k) {
					shapes[i].push_back(shape[k]);
				}
			}
		}
	}
	*/
	
	// detect close vertices
	{
		time_t start = clock();
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = vertices(roads.graph); vi != vend; ++vi) {
			if (!roads.graph[*vi]->valid) continue;
			if (usedVertices.contains(*vi)) continue;

			// don't start from the non-intersection.
			if (GraphUtil::getDegree(roads, *vi) <= 2) continue;

			std::vector<RoadEdgeDesc> shape;
			addVerticesToGroup(roads, *vi, threshold, shape, usedVertices, usedEdges);
			shapes.push_back(shape);
		}
		time_t end = clock();
		std::cout << "Close vertices detection: " << (double)(end - start) / CLOCKS_PER_SEC << " [sec]" << std::endl;
	}

	

	return shapes;
}

RoadEdgeDescs ShapeDetector::addVerticesToCircle(RoadGraph &roads, RoadVertexDesc srcDesc, float threshold, QMap<RoadVertexDesc, bool> &visited, QMap<RoadEdgeDesc, int> &usedEdges) {
	RoadEdgeDescs shape;

	float threshold2 = SQR(threshold);

	std::list<RoadVertexDesc> queue;
	queue.push_back(srcDesc);

	QMap<RoadVertexDesc, bool> vertex_descs;
	vertex_descs[srcDesc] = true;

	QMap<RoadEdgeDesc, bool> edge_descs;

	while (!queue.empty()) {
		RoadVertexDesc desc = queue.front();
		queue.pop_front();
		visited[desc] = true;

		if (GraphUtil::getDegree(roads, desc) > 2) continue;

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(desc, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;
			//if (edge_descs.contains(*ei)) continue;
			if (usedEdges.contains(*ei) && usedEdges[*ei] > 1) continue;

			RoadVertexDesc tgt = boost::target(*ei, roads.graph);
			if (visited.contains(tgt)) continue;

			//if (GraphUtil::getDegree(roads, tgt) <= 2 || (roads.graph[desc]->pt - roads.graph[tgt]->pt).lengthSquared() < threshold2) {
				edge_descs[*ei] = true;

				// GEN 2014/8/26 fixed the bug
				if (usedEdges.contains(*ei)) {
					usedEdges[*ei]++;
				} else {
					usedEdges[*ei] = 1;
				}

				if (!vertex_descs.contains(tgt)) {				
					queue.push_back(tgt);
					vertex_descs[tgt] = true;
				}
			//}
		}
	}

	for (QMap<RoadEdgeDesc, bool>::iterator it = edge_descs.begin(); it != edge_descs.end(); ++it) {
		shape.push_back(it.key());
	}

	return shape;
}


void ShapeDetector::addVerticesToGroup(RoadGraph &roads, RoadVertexDesc srcDesc, float threshold, RoadEdgeDescs &shape, QMap<RoadVertexDesc, bool> &usedVertices, QMap<RoadEdgeDesc, int> &usedEdges) {
	std::cout << "shape is detected..." << srcDesc << std::endl;

	// パッチに含まれるエッジのセット
	QMap<RoadEdgeDesc, bool> edge_descs;

	QMap<RoadVertexDesc, bool> visited;

	std::list<RoadVertexDesc> queue;
	queue.push_back(srcDesc);
	usedVertices[srcDesc] = true;
	visited[srcDesc] = true;
	roads.graph[srcDesc]->properties["length"] = 0.0f;

	while (!queue.empty()) {
		RoadVertexDesc desc = queue.front();
		queue.pop_front();

		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(desc, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;
			//if (usedEdges.contains(*ei)) continue;

			usedEdges[*ei] = 1;
			edge_descs[*ei] = true;
			RoadVertexDesc tgt = boost::target(*ei, roads.graph);
			if (usedVertices.contains(tgt)) continue;
			if (visited.contains(tgt)) continue;

			if (GraphUtil::getDegree(roads, tgt) <= 2 || roads.graph[desc]->properties["length"].toFloat() + roads.graph[*ei]->polyline.length() <= threshold) {
				queue.push_back(tgt);
				visited[tgt] = true;

				if (GraphUtil::getDegree(roads, tgt) <= 2) {
					roads.graph[tgt]->properties["length"] = roads.graph[desc]->properties["length"].toFloat() + roads.graph[*ei]->polyline.length();
				} else {
					roads.graph[tgt]->properties["length"] = 0.0f;
					usedVertices[tgt] = true;
				}

				/*
				// GEN 2014/8/26 fixed the bug
				if (usedEdges.contains(*ei)) {
					usedEdges[*ei]++;
				} else {
					usedEdges[*ei] = 1;
				}
				usedVertices[tgt] = true;

				if (!vertex_descs.contains(tgt)) {				
					queue.push_back(tgt);
					vertex_descs[tgt] = true;
				}
				*/
			}
		}
	}

	for (QMap<RoadEdgeDesc, bool>::iterator it = edge_descs.begin(); it != edge_descs.end(); ++it) {
		shape.push_back(it.key());
	}
}
