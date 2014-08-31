#include "PatchRoadGenerator.h"
#include <QTest>
#include <boost/graph/planar_face_traversal.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include "global.h"
#include "Util.h"
#include "ConvexHull.h"
#include "GraphUtil.h"
#include "RoadGeneratorHelper.h"
#include "SmallBlockRemover.h"
#include "ShapeDetector.h"

void PatchRoadGenerator::generateRoadNetwork() {
	srand(12345);

	// mark all the existing vertices as fixed
	{
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			roads.graph[*vi]->fixed = true;
		}
	}

	// remove all the local streets temporalily
	QMap<RoadEdgeDesc, bool> originalLocalStreets;
	{
		RoadEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;
			if (roads.graph[*ei]->type == RoadEdge::TYPE_STREET) {
				originalLocalStreets[*ei] = true;
				roads.graph[*ei]->valid = false;
			}
		}
	}

	std::list<RoadVertexDesc> seeds;

	int video_frame_id = 0;

	// Avenueのシードを生成
	generateAvenueSeeds(seeds);

	// Avenueを生成
	std::cout << "Avenue generation started." << std::endl;
	{
		// detect interesting shapes
		std::vector<std::vector<RoadEdgeDescs> > shapes;
		std::vector<std::vector<Patch> > patches;
		{
			shapes.resize(features.size());
			patches.resize(features.size());
			for (int i = 0; i < features.size(); ++i) {
				shapes[i] = features[i].shapes(RoadEdge::TYPE_AVENUE, G::getFloat("houghScale"), G::getFloat("avenuePatchDistance"));
				patches[i] = RoadGeneratorHelper::convertToPatch(RoadEdge::TYPE_AVENUE, features[i].reducedRoads(RoadEdge::TYPE_AVENUE), shapes[i]);

				// 各エッジに、shape_idをセットする
				// ExFeature側で、patchIdはセット済み。なので、これは不要！
				/*
				for (int j = 0; j < shapes[i].size(); ++j) {
					for (int k = 0 ; k < shapes[i][j].size(); ++k) {
						features[i].reducedRoads(RoadEdge::TYPE_AVENUE).graph[shapes[i][j][k]]->properties["shape_id"] = j;
					}
				}
				*/
			}
		}

		int iter;
		for (iter = 0; !seeds.empty() && iter < G::getInt("numAvenueIterations"); ) {
			RoadVertexDesc desc = seeds.front();
			seeds.pop_front();

			// エリアの外なら、スキップする
			if (!targetArea.contains(roads.graph[desc]->pt)) {
				roads.graph[desc]->onBoundary = true;
				continue;
			}

			// 水中なら、スキップする
			float z = vboRenderManager->getTerrainHeight(roads.graph[desc]->pt.x(), roads.graph[desc]->pt.y(), true);
			if (z < G::getFloat("seaLevelForAvenue")) {
				std::cout << "attemptExpansion (avenue): " << iter << " (skipped because it is under the sea)" << std::endl;
				continue;
			}

			std::cout << "attemptExpansion (avenue): " << iter << " (Seed: " << desc << ")" << std::endl;
			int ex_id = roads.graph[desc]->properties["ex_id"].toInt();
			if (!attemptExpansion(RoadEdge::TYPE_AVENUE, desc, features[ex_id], patches[ex_id], seeds)) {
				attemptExpansion2(RoadEdge::TYPE_AVENUE, desc, features[ex_id], seeds);
			}

			char filename[255];
			sprintf(filename, "road_images/avenues_%d.jpg", iter);
			saveRoadImage(roads, filename);

			iter++;
		}
	}
	std::cout << "Avenue generation completed." << std::endl;

	seeds.clear();

	// Avenueをクリーンナップ
	if (G::getBool("cleanAvenues")) {
		RoadGeneratorHelper::extendDanglingEdges(roads, 300.0f);
		RoadGeneratorHelper::removeDeadend(roads);
		//GraphUtil::reduce(roads);
		//GraphUtil::removeLoop(roads);
	}

	if (G::getBool("removeSmallBlocks")) {
		SmallBlockRemover::remove(roads, G::getFloat("minBlockSize"));
	}

	// recover the temporarily removed local streets if they are not intersected with other edges
	{
		for (QMap<RoadEdgeDesc, bool>::iterator it = originalLocalStreets.begin(); it != originalLocalStreets.end(); ++it) {
			if (!GraphUtil::isIntersect(roads, roads.graph[it.key()]->polyline)) {
				roads.graph[it.key()]->valid = true;
			}
		}
	}

	// Local streetを生成
	if (G::getBool("generateLocalStreets")) {
		generateStreetSeeds(seeds);
		
		// detect interesting shapes
		std::vector<std::vector<RoadEdgeDescs> > shapes;
		std::vector<std::vector<Patch> > patches;
		{
			shapes.resize(features.size());
			patches.resize(features.size());
			for (int i = 0; i < features.size(); ++i) {
				shapes[i] = features[i].shapes(RoadEdge::TYPE_STREET, G::getFloat("houghScale"), G::getFloat("streetPatchDistance"));
				patches[i] = RoadGeneratorHelper::convertToPatch(RoadEdge::TYPE_AVENUE, features[i].reducedRoads(RoadEdge::TYPE_AVENUE), shapes[i]);

				for (int j = 0; j < shapes[i].size(); ++j) {
					for (int k = 0 ; k < shapes[i][j].size(); ++k) {
						features[i].reducedRoads(RoadEdge::TYPE_STREET).graph[shapes[i][j][k]]->properties["shape_id"] = j;
					}
				}
			}
		}
		
		std::cout << "Local street generation started." << std::endl;

		int i;
		for (i = 0; !seeds.empty() && i < G::getInt("numStreetIterations"); ++i) {
			RoadVertexDesc desc = seeds.front();
			seeds.pop_front();

			float z = vboRenderManager->getTerrainHeight(roads.graph[desc]->pt.x(), roads.graph[desc]->pt.y(), true);
			if (z < G::getFloat("seaLevelForStreet")) {
				std::cout << "attemptExpansion (street): " << i << " (skipped because it is under the sea or on the mountains)" << std::endl;
				continue;
			}

			std::cout << "attemptExpansion (street): " << i << " (Seed: " << desc << ")" << std::endl;
			if (roads.graph[desc]->properties["generation_type"] == "snapped") continue;

			int ex_id = roads.graph[desc]->properties["ex_id"].toInt();
			attemptExpansion(RoadEdge::TYPE_STREET, desc, features[ex_id], patches[ex_id], seeds);
		}
		std::cout << "Local street generation completed." << std::endl;
	}

	// 指定されたエリアでCropping
	if (G::getBool("cropping")) {
		GraphUtil::extractRoads2(roads, targetArea);
	}

	if (G::getBool("cleanStreets")) {
		RoadGeneratorHelper::removeDeadend(roads);
	}
	
	GraphUtil::cleanEdges(roads);
}

/**
 * シード頂点を生成する。
 */
void PatchRoadGenerator::generateAvenueSeeds(std::list<RoadVertexDesc>& seeds) {
	seeds.clear();

	int numSeedsPerEx = hintLine.size() / features.size();

	for (int i = 0; i <features.size(); ++i) {
		for (int j = 0; j < numSeedsPerEx; ++j) {
			int index = i * numSeedsPerEx + j;
			addAvenueSeed(features[i], hintLine[index], features[i].hintLine[j], index, i, seeds);
		}
	}
}

/**
 * Avenue用のシードを座標pt付近に追加する。
 * 座標pt付近の、該当するカーネルを捜し、そのカーネルを使ってシードを追加する。
 *
 * @param f				特徴量
 * @param pt			シード座標
 * @param seeds			追加されたシードは、seedsに追加される。
 */
bool PatchRoadGenerator::addAvenueSeed(ExFeature &f, const QVector2D &pt, const QVector2D &ex_pt, int group_id, int ex_id, std::list<RoadVertexDesc>& seeds) {
	if (!targetArea.contains(pt)) return false;

	RoadVertexDesc seedDesc = GraphUtil::getVertex(f.reducedRoads(RoadEdge::TYPE_AVENUE), ex_pt);

	// 頂点を追加し、シードとする
	RoadVertexPtr v = RoadVertexPtr(new RoadVertex(pt));
	RoadVertexDesc desc = GraphUtil::addVertex(roads, v);
	//roads.graph[desc]->properties["used"] = true;
	roads.graph[desc]->properties["group_id"] = group_id;
	roads.graph[desc]->properties["generation_type"] = "example";
	roads.graph[desc]->properties["ex_id"] = ex_id;
	roads.graph[desc]->properties["example_desc"] = seedDesc;
	roads.graph[desc]->properties["rotation_angle"] = G::getFloat("rotationAngle");//0.0f;
	seeds.push_back(desc);

	return true;
}

/**
 * Local Street用のシードを生成する。
 */
void PatchRoadGenerator::generateStreetSeeds(std::list<RoadVertexDesc> &seeds) {
	seeds.clear();

	// 頂点自体を、Local streetsのシードにする
	{
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
			std::cout << "Initial seed generation for the local street (vertex: " << (*vi) << ")" << std::endl;

			if (!roads.graph[*vi]->valid) continue;

			// The surrounding roads shoule not be used for the local street genration any more.
			if (roads.graph[*vi]->fixed) continue;

			if (!roads.graph[*vi]->properties.contains("example_desc")) continue;

			if (!targetArea.contains(roads.graph[*vi]->pt)) continue;
			float z = vboRenderManager->getTerrainHeight(roads.graph[*vi]->pt.x(), roads.graph[*vi]->pt.y(), true);
			if (z < G::getFloat("seaLevelForStreet")) continue;

			// ターゲットエリア座標空間から、Example座標空間へのオフセットを計算
			int ex_id = roads.graph[*vi]->properties["ex_id"].toInt();
			RoadVertexDesc ex_v_desc = roads.graph[*vi]->properties["example_desc"].toUInt();

			RoadVertexDesc seedDesc;
			if (GraphUtil::getVertex(features[ex_id].reducedRoads(RoadEdge::TYPE_STREET), features[ex_id].reducedRoads(RoadEdge::TYPE_AVENUE).graph[ex_v_desc]->pt, 1.0f, seedDesc)) {
				seeds.push_back(*vi);
				roads.graph[*vi]->properties["example_street_desc"] = seedDesc;
			}

		}
	}
}

/**
 * このシードを使って、道路生成する。
 * Exampleベースで生成する。
 */
bool PatchRoadGenerator::attemptExpansion(int roadType, RoadVertexDesc srcDesc, ExFeature& f, std::vector<Patch> &patches, std::list<RoadVertexDesc> &seeds) {
	RoadVertexDesc ex_v_desc;
	if (roadType == RoadEdge::TYPE_AVENUE) {
		ex_v_desc = roads.graph[srcDesc]->properties["example_desc"].toUInt();
	} else {
		ex_v_desc = roads.graph[srcDesc]->properties["example_street_desc"].toUInt();
	}
	float angle = roads.graph[srcDesc]->properties["rotation_angle"].toFloat();
	RoadVertexPtr ex_vertex = f.reducedRoads(roadType).graph[ex_v_desc];

	int patch_id = ex_vertex->patchId;

	RoadGraph replacementGraph;

	// 元のExampleで境界上の頂点である場合、対応するパッチがない。
	// そこで、適当なパッチを探す。
	if (ex_vertex->onBoundary) {
		Polyline2D polyline = GraphUtil::getAdjoiningPolyline(roads, srcDesc);

		float min_cost = std::numeric_limits<float>::max();
		RoadVertexDesc root_desc;
		RoadVertexDesc connect_desc;

		std::reverse(polyline.begin(), polyline.end());
		for (int i = 0; i < patches.size(); ++i) {
			// 現在の頂点が属するパッチと、同じパッチは、使わない。
			// もし使っちゃったら、同じパッチが並んでしまう。
			if (i == roads.graph[srcDesc]->patchId) continue;

			RoadVertexDesc v_connector;
			RoadVertexDesc v_root;
			float cost = patches[i].getMinCost(polyline, v_connector, v_root);
			if (cost < min_cost) {
				min_cost = cost;
				patch_id = i;
				connect_desc = v_connector;
				root_desc = v_root;
			}
		}

		// streetの場合は、example_street_descにしないといけないか？
		// いや、大丈夫。streetのパッチは、元のgraphの頂点IDを、やはりexample_descに格納しているから。
		ex_v_desc = patches[patch_id].roads.graph[root_desc]->properties["example_desc"].toUInt();


		buildReplacementGraphByExample(roadType, replacementGraph, srcDesc, f.reducedRoads(roadType), ex_v_desc, angle, patches[patch_id], patch_id, connect_desc);

	} else {
		if (patch_id < 0) {
			printf("ERROR!!!!!!!!!!!!!!");
		}
	
		buildReplacementGraphByExample(roadType, replacementGraph, srcDesc, f.reducedRoads(roadType), ex_v_desc, angle, patches[patch_id], patch_id);
	}

	// replacementGraphが、既に生成済みのグラフと重なるかどうかチェック
	if (checkCrossing(replacementGraph)) {
		int xxx = 0;

		return false;
	}
	

	rewrite(roadType, srcDesc, replacementGraph, seeds);

	return true;
}

void PatchRoadGenerator::buildReplacementGraphByExample(int roadType, RoadGraph &replacementGraph, RoadVertexDesc srcDesc, RoadGraph &exRoads, RoadVertexDesc ex_srcDesc, float angle, Patch &patch, int patchId) {
	replacementGraph.clear();

	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// add vertices of the patch
	{
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(patch.roads.graph); vi != vend; ++vi) {
			if (patch.roads.graph[*vi]->onBoundary) {
				int xxx = 0;
			}

			// Transformした後の座標を計算
			QVector2D pt = Util::transform(patch.roads.graph[*vi]->pt, exRoads.graph[ex_srcDesc]->pt, angle, roads.graph[srcDesc]->pt);

			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(*patch.roads.graph[*vi]));
			v->pt = pt;
			v->properties["rotation_angle"] = angle;
			v->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
			v->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
			if (v->patchId != patchId) {
				int xxxx = 0;
			}
			v->patchId = patchId;

			// 元のExampleで境界上の頂点だったかどうかは、ターゲットエリアでは関係ないので、一旦リセットする
			v->onBoundary = false;

			RoadVertexDesc v_desc = GraphUtil::addVertex(replacementGraph, v);
			conv[*vi] = v_desc;
		}
	}

	// patchのエッジを追加
	{
		RoadEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::edges(patch.roads.graph); ei != eend; ++ei) {
			RoadVertexDesc src = boost::source(*ei, patch.roads.graph);
			RoadVertexDesc tgt = boost::target(*ei, patch.roads.graph);

			RoadEdgePtr edge = RoadEdgePtr(new RoadEdge(*patch.roads.graph[*ei]));

			// エッジを座標変換する
			edge->polyline.translate(-exRoads.graph[ex_srcDesc]->pt);
			edge->polyline.rotate(Util::rad2deg(-angle));
			edge->polyline.translate(roads.graph[srcDesc]->pt);

			RoadEdgeDesc e_desc = GraphUtil::addEdge(replacementGraph, conv[src], conv[tgt], edge);
		}
	}
}

void PatchRoadGenerator::buildReplacementGraphByExample(int roadType, RoadGraph &replacementGraph, RoadVertexDesc srcDesc, RoadGraph &exRoads, RoadVertexDesc ex_srcDesc, float angle, Patch &patch, int patchId, RoadVertexDesc v_connect) {
	replacementGraph.clear();

	QMap<RoadVertexDesc, RoadVertexDesc> conv;
	RoadVertexDesc connector_desc;

	// add vertices of the patch
	{
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(patch.roads.graph); vi != vend; ++vi) {
			if (patch.roads.graph[*vi]->onBoundary) {
				int xxx = 0;
			}

			// Transformした後の座標を計算
			QVector2D pt = Util::transform(patch.roads.graph[*vi]->pt, exRoads.graph[ex_srcDesc]->pt, angle, roads.graph[srcDesc]->pt);

			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(*patch.roads.graph[*vi]));
			v->pt = pt;
			v->properties["rotation_angle"] = angle;
			v->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
			v->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
			if (v->patchId != patchId) {
				int xxxx = 0;
			}
			v->patchId = patchId;

			// 元のExampleで境界上の頂点だったかどうかは、ターゲットエリアでは関係ないので、一旦リセットする
			v->onBoundary = false;

			RoadVertexDesc v_desc = GraphUtil::addVertex(replacementGraph, v);
			conv[*vi] = v_desc;

			if (*vi == v_connect) {
				connector_desc = v_desc;
			}
		}
	}

	// patchのエッジを追加
	{
		RoadEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::edges(patch.roads.graph); ei != eend; ++ei) {
			RoadVertexDesc src = boost::source(*ei, patch.roads.graph);
			RoadVertexDesc tgt = boost::target(*ei, patch.roads.graph);

			RoadEdgePtr edge = RoadEdgePtr(new RoadEdge(*patch.roads.graph[*ei]));

			// エッジを座標変換する
			edge->polyline.translate(-exRoads.graph[ex_srcDesc]->pt);
			edge->polyline.rotate(Util::rad2deg(-angle));
			edge->polyline.translate(roads.graph[srcDesc]->pt);

			RoadEdgeDesc e_desc = GraphUtil::addEdge(replacementGraph, conv[src], conv[tgt], edge);
		}
	}

	// 指定されたコネクタからのエッジを削除
	{
		QMap<RoadVertexDesc, bool> visited;
		std::list<RoadVertexDesc> queue;
		queue.push_back(connector_desc);

		while (!queue.empty()) {
			RoadVertexDesc v = queue.front();
			queue.pop_front();

			RoadOutEdgeIter ei, eend;
			for (boost::tie(ei, eend) = boost::out_edges(v, replacementGraph.graph); ei != eend; ++ei) {
				if (!replacementGraph.graph[*ei]->valid) continue;

				replacementGraph.graph[*ei]->valid = false;

				RoadVertexDesc tgt = boost::target(*ei, replacementGraph.graph);
				if (visited[tgt]) continue;

				// 上で既に一本のエッジを無効にしているので、もともとdegree=2の頂点は、残り一本だけエッジが残っているはず。
				// なので、 == 2　ではなく、 == 1　とする。
				if (GraphUtil::getDegree(replacementGraph, tgt) == 1) {
					queue.push_back(tgt);
				}
			}
		}
	}

	GraphUtil::clean(replacementGraph);
}

void PatchRoadGenerator::rewrite(int roadType, RoadVertexDesc srcDesc, RoadGraph &replacementGraph, std::list<RoadVertexDesc>& seeds) {
	QMap<RoadVertexDesc, RoadVertexDesc> conv;

	// add vertices
	{
		RoadVertexIter vi, vend;
		for (boost::tie(vi, vend) = boost::vertices(replacementGraph.graph); vi != vend; ++vi) {
			if (!replacementGraph.graph[*vi]->valid) continue;

			RoadVertexPtr v = RoadVertexPtr(new RoadVertex(*replacementGraph.graph[*vi]));
			RoadVertexDesc v_desc;
			if (!GraphUtil::getVertex(roads, replacementGraph.graph[*vi]->pt, 1.0f, v_desc)) {
				// 頂点を新規追加
				v_desc = GraphUtil::addVertex(roads, v);

				// 新規追加された頂点について、もとのreplacementGraphでdegree==1で、deadendでないなら、シードに追加
				if (GraphUtil::getDegree(replacementGraph, *vi) == 1 && replacementGraph.graph[*vi]->deadend == false) {
					seeds.push_back(v_desc);
				}
			} else {
				roads.graph[v_desc]->properties = replacementGraph.graph[*vi]->properties;

				// initial seed自体がrewriteされる場合、seedに入れないと、そこからエッジが延びていかない
				//if (GraphUtil::getDegree(roads, v_desc) == 0) {
				//	seeds.push_back(v_desc);
				//}
			}

			conv[*vi] = v_desc;
		}
	}
	
	// add edges
	{
		RoadEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::edges(replacementGraph.graph); ei != eend; ++ei) {
			if (!replacementGraph.graph[*ei]->valid) continue;

			RoadVertexDesc src = boost::source(*ei, replacementGraph.graph);
			RoadVertexDesc tgt = boost::target(*ei, replacementGraph.graph);

			if (!GraphUtil::hasEdge(roads, conv[src], conv[tgt])) {
				RoadEdgePtr e = RoadEdgePtr(new RoadEdge(*replacementGraph.graph[*ei]));
				GraphUtil::addEdge(roads, conv[src], conv[tgt], e);
			}
		}
	}
}

/**
 * このシードを使って、PM方式で道路生成する。
 * パッチが使えないケースに使用される。
 * 通常、パッチ間の接着剤として使いたいなぁ。。。
 */
void PatchRoadGenerator::attemptExpansion2(int roadType, RoadVertexDesc srcDesc, ExFeature& f, std::list<RoadVertexDesc> &seeds) {
	float snapThreshold;

	if (roadType == RoadEdge::TYPE_AVENUE) {
		snapThreshold = f.avgAvenueLength * 0.2f;
	} else {
		snapThreshold = f.avgStreetLength * 0.2f;
	}

	float roadSnapFactor = G::getFloat("roadSnapFactor");
	float roadAngleTolerance = G::getFloat("roadAngleTolerance");

	std::vector<RoadEdgePtr> edges;

	// 当該頂点から出るエッジをリストアップし、基底の方向を決定する
	float direction = 0.0f;
	{
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(srcDesc, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			Polyline2D polyline  = GraphUtil::orderPolyLine(roads, *ei, srcDesc);
			QVector2D vec = polyline[1] - polyline[0];
			direction = atan2f(vec.y(), vec.x());
			break;
		}
	}

	// 既にあるエッジと正反対の方向を計算
	direction += 3.141592653;

	// ものすごい近くに、他の頂点がないかあれば、そことコネクトして終わり。
	// それ以外の余計なエッジは生成しない。さもないと、ものすごい密度の濃い道路網になっちゃう。
	RoadVertexDesc nearestDesc;
	if (GraphUtil::getVertex(roads, srcDesc, 100.0f, direction, 1.5f, nearestDesc)) {
		// もし、既にエッジがあるなら、キャンセル
		if (GraphUtil::hasEdge(roads, srcDesc, nearestDesc)) return;

		RoadEdgePtr e = RoadEdgePtr(new RoadEdge(roadType, 1));
		e->polyline.push_back(roads.graph[srcDesc]->pt);
		e->polyline.push_back(roads.graph[nearestDesc]->pt);

		RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, srcDesc, nearestDesc, e);
	} else {
		// 道路生成用のカーネルを合成する
		synthesizeItem(roadType, srcDesc, edges);
	}

	float z = vboRenderManager->getTerrainHeight(roads.graph[srcDesc]->pt.x(), roads.graph[srcDesc]->pt.y(), true);

	int cnt = 0;
	for (int i = 0; i < edges.size(); ++i) {
		if (RoadGeneratorHelper::isRedundantEdge(roads, srcDesc, edges[i]->polyline, roadAngleTolerance)) continue;
		if (growRoadSegment2(roadType, srcDesc, f, edges[i]->polyline, edges[i]->lanes, 0, false, roadSnapFactor, roadAngleTolerance, seeds)) {
			cnt++;
		}

		// hack: to avoid creating an intersection on the river.
		if (roadType == RoadEdge::TYPE_AVENUE && z < G::getFloat("seaLevelForStreet")) {
			if (cnt >= 1) break;
		}

	}
}

/**
 * 指定されたpolylineに従って、srcDesc頂点からエッジを伸ばす。
 * エッジの端点が、srcDescとは違うセルに入る場合は、falseを返却する。
 */
bool PatchRoadGenerator::growRoadSegment2(int roadType, RoadVertexDesc srcDesc, ExFeature& f, const Polyline2D &polyline, int lanes, RoadVertexDesc next_ex_v_desc, bool byExample, float snapFactor, float angleTolerance, std::list<RoadVertexDesc> &seeds) {
	float angle = atan2f(polyline[1].y() - polyline[0].y(), polyline[1].x() - polyline[0].x());

	RoadEdgePtr new_edge = RoadEdgePtr(new RoadEdge(roadType, lanes));
	new_edge->polyline.push_back(roads.graph[srcDesc]->pt);

	RoadVertexDesc tgtDesc;
	bool found = false;
	{
		// 指定された方向で、直近の頂点があるかチェックする。
		// まずは、指定された方向に非常に近い頂点があるかチェックする。この際、距離は少し遠くまで許容する。
		// 次に、方向のレンジを少し広げて、その代わり、距離を短くして、改めてチェックする。
		if (GraphUtil::getVertex(roads, srcDesc, 500.0f, angle, 0.1f, tgtDesc)) {
			found = true;
		} else if (GraphUtil::getVertex(roads, srcDesc, 400.0f, angle, 0.2f, tgtDesc)) {
			found = true;
		} else if (GraphUtil::getVertex(roads, srcDesc, 300.0f, angle, 0.3f, tgtDesc)) {
			found = true;
		}

		if (found) {
			// もしスナップ先が、シードじゃないなら、エッジ生成をキャンセル
			if (std::find(seeds.begin(), seeds.end(), tgtDesc) == seeds.end()) {
				//（要検討。80%の確率ぐらいにすべきか？)
				if (Util::genRand(0, 1) < 0.8f) return false;
			}

			// もしスナップ先の頂点が、redundantなエッジを持っているなら、エッジ生成をキャンセル
			Polyline2D snapped_polyline;
			snapped_polyline.push_back(QVector2D(0, 0));
			snapped_polyline.push_back(QVector2D(roads.graph[srcDesc]->pt - roads.graph[tgtDesc]->pt));
			if (RoadGeneratorHelper::isRedundantEdge(roads, tgtDesc, snapped_polyline, 0.3f)) {
				//（とりあえず、ものすごい鋭角の場合は、必ずキャンセル)
				return false;
			}

			new_edge->polyline.push_back(roads.graph[tgtDesc]->pt);
		}
	}

	if (!found) {
		// snap先がなければ、指定されたpolylineに従ってエッジを生成する。
		new_edge->polyline.push_back(roads.graph[srcDesc]->pt + polyline[1]);

		// もし、新規エッジが、既存グラフと交差するなら、エッジ生成をキャンセル
		QVector2D intPoint;
		if (GraphUtil::isIntersect(roads, new_edge->polyline, srcDesc, intPoint)) {
			// 60%の確率でキャンセル？
			if (Util::genRand(0, 1) < 0.6f) return false;

			// 交差する箇所で中断させる
			new_edge->polyline[1] = intPoint;

			// 直近のエッジを取得
			RoadEdgeDesc closestEdge;
			RoadGeneratorHelper::getCloseEdge(roads, intPoint, false, roads.graph[srcDesc]->properties["group_id"].toInt(), 1.0f, srcDesc, closestEdge, intPoint);

			// 他のエッジにスナップ
			tgtDesc = GraphUtil::splitEdge(roads, closestEdge, intPoint);
			roads.graph[tgtDesc]->properties["generation_type"] = "snapped";
			roads.graph[tgtDesc]->properties["group_id"] = roads.graph[closestEdge]->properties["group_id"];
			roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[closestEdge]->properties["ex_id"];
			roads.graph[tgtDesc]->properties.remove("example_desc");

			found = true;
		}
	}

	if (!found) {
		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(new_edge->polyline.last()));
		tgtDesc = GraphUtil::addVertex(roads, v);

		seeds.push_back(tgtDesc);
	}
	
	// エッジを追加
	RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, srcDesc, tgtDesc, new_edge);


	/*
	float sigma2 = SQR(G::getFloat("interpolationSigma1"));
	float threshold1 = G::getFloat("interpolationThreshold1");

	bool intercepted = false;

	float z0 = vboRenderManager->getTerrainHeight(roads.graph[srcDesc]->pt.x(), roads.graph[srcDesc]->pt.y(), true);

	// 新しいエッジを生成
	RoadEdgePtr new_edge = RoadEdgePtr(new RoadEdge(roadType, lanes));
	QVector2D pt = roads.graph[srcDesc]->pt;
	for (int i = 0; i < polyline.size(); ++i) {
		if (i > 0) {
			pt += polyline[i] - polyline[i - 1];
		}
		//QVector2D pt = roads.graph[srcDesc]->pt + polyline[i];

		new_edge->polyline.push_back(pt);

		// 水没、または、山の上なら、道路生成をストップ
		float z = vboRenderManager->getTerrainHeight(pt.x(), pt.y(), true);
		if ((roadType == RoadEdge::TYPE_AVENUE && z < G::getFloat("seaLevelForAvenue")) || (roadType == RoadEdge::TYPE_STREET && z < G::getFloat("seaLevelForStreet"))) {
			// 最初っから水没している場合は、そもそもエッジ生成をキャンセル
			if (new_edge->polyline.size() <= 1) return false;

			RoadGeneratorHelper::bendEdgeBySteepElevationChange(roadType, new_edge->polyline, vboRenderManager);
			pt = new_edge->polyline.back();

			intercepted = true;
		}

		// if the slope is too steep, bend the direction to avoid climbing up
		if (z0 >= 70.0f && fabs(z - z0) > 7.0f) {
			RoadGeneratorHelper::bendEdgeBySteepElevationChange(new_edge->polyline, z0, vboRenderManager);
			pt = new_edge->polyline.back();
			z = vboRenderManager->getTerrainHeight(pt.x(), pt.y(), true);
			z0 = z;
		}

		// 他のエッジと交差したら、道路生成をストップ
		QVector2D intPoint;
		//if (roadType == RoadEdge::TYPE_STREET && GraphUtil::isIntersect(roads, new_edge->polyline, intPoint)) {
		if (GraphUtil::isIntersect(roads, new_edge->polyline, srcDesc, intPoint)) {
			new_edge->polyline.erase(new_edge->polyline.begin() + new_edge->polyline.size() - 1);
			new_edge->polyline.push_back(intPoint);

			intercepted = true;

			// エッジ長が最短thresholdより短い場合は、キャンセル
			if (new_edge->getLength() < 30.0f) return false;

			// エッジがgoingStraightではなく、且つ、もとのエッジ長の半分未満にinterceptされた場合は、キャンセル
			//if (!goingStraight && new_edge->getLength() < polyline.length() * 0.5f) return false;
		}

		if (intercepted) break;
	}

	if (new_edge->polyline.size() == 1) return false;

	RoadVertexDesc tgtDesc;

	if (byExample) {
		//snapFactor = 0.01f;
	}

	bool snapTried = false;
	bool snapDone = false;
	RoadEdgeDesc closestEdge;
	QVector2D intPoint;

	// snap to myself (loop edge)
	if ((new_edge->polyline.last() - roads.graph[srcDesc]->pt).lengthSquared() < 0.1f) {
		tgtDesc = srcDesc;
		snapTried = true;
		snapDone = true;
	}
	
	if (!snapDone && RoadGeneratorHelper::getCloseVertex(roads, new_edge->polyline.last(), byExample, roads.graph[srcDesc]->properties["group_id"].toInt(), (new_edge->polyline.back() - new_edge->polyline[0]).length() * snapFactor, srcDesc, tgtDesc)) {
		snapTried = true;

		if (byExample && roads.graph[tgtDesc]->properties["generation_type"] == "example" && roads.graph[tgtDesc]->properties["group_id"] == roads.graph[srcDesc]->properties["group_id"]) {
			angleTolerance = 0.01f;
		}

		// 他の頂点にスナップ
		Polyline2D new_polyline = new_edge->polyline;
		GraphUtil::movePolyline(roads, new_polyline, roads.graph[srcDesc]->pt, roads.graph[tgtDesc]->pt);

		if (RoadGeneratorHelper::submerged(roadType, new_polyline, vboRenderManager)) {
			new_polyline = new_edge->polyline;
			new_polyline.push_back(roads.graph[tgtDesc]->pt);
		}

		// 他のエッジと交差するか
		if (!GraphUtil::isIntersect(roads, new_polyline)) {
			std::reverse(new_polyline.begin(), new_polyline.end());
			if (!GraphUtil::hasRedundantEdge(roads, tgtDesc, new_polyline, angleTolerance)) {
				new_edge->polyline = new_polyline;
				snapDone = true;
			}
		}
	}
	
	if (!snapDone && RoadGeneratorHelper::getCloseEdge(roads, new_edge->polyline.last(), byExample, roads.graph[srcDesc]->properties["group_id"].toInt(), (new_edge->polyline.back() - new_edge->polyline[0]).length() * snapFactor, srcDesc, closestEdge, intPoint)) {
		snapTried = true;

		if (byExample && roads.graph[closestEdge]->properties["generation_type"] == "example" && roads.graph[closestEdge]->properties["group_id"] == roads.graph[srcDesc]->properties["group_id"]) {
			angleTolerance = 0.01f;
		}

		// 他のエッジにスナップ
		tgtDesc = GraphUtil::splitEdge(roads, closestEdge, intPoint);
		roads.graph[tgtDesc]->properties["generation_type"] = "snapped";
		roads.graph[tgtDesc]->properties["group_id"] = roads.graph[closestEdge]->properties["group_id"];
		roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[closestEdge]->properties["ex_id"];
		roads.graph[tgtDesc]->properties.remove("example_desc");

		GraphUtil::movePolyline(roads, new_edge->polyline, roads.graph[srcDesc]->pt, roads.graph[tgtDesc]->pt);
		if (!GraphUtil::isIntersect(roads, new_edge->polyline)) {
			if (!GraphUtil::hasRedundantEdge(roads, tgtDesc, new_edge->polyline, angleTolerance)) {
				snapDone = true;
				roads.graph[tgtDesc]->properties["parent"] = srcDesc;
			}
		}
	}

	if (snapTried && !snapDone) return false;

	if (!snapDone) {
		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(new_edge->polyline.last()));
		tgtDesc = GraphUtil::addVertex(roads, v);
		roads.graph[tgtDesc]->properties["parent"] = srcDesc;

		// 新しい頂点にgroup_id、ex_idを引き継ぐ
		roads.graph[tgtDesc]->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
		roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];

		// 新しい頂点のgeneration_typeを設定
		roads.graph[tgtDesc]->properties["generation_type"] = byExample ? "example" : "mixed";

		if (targetArea.contains(new_edge->polyline.last())) {
			roads.graph[tgtDesc]->properties["generation_type"] = "mixed";

			// シードに追加する
			if (byExample && !intercepted) {
				//if (roadType == RoadEdge::TYPE_AVENUE || GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
				if (f.reducedRoads(roadType).graph[next_ex_v_desc]->onBoundary || GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
					seeds.push_back(tgtDesc);
				}

				// 対応するExampleが存在する場合は、それを設定する
				//if (!f.reducedRoads(roadType).graph[next_ex_v_desc]->properties.contains("used") && GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
				if (GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {

					// 初期シード位置から離れると、mixedに変わる確率が上がる
					float D = sqrtf(f.area.area());
					float x_ratio = (roads.graph[tgtDesc]->pt.x() - hintLine[roads.graph[tgtDesc]->properties["group_id"].toInt()].x()) / D;
					float y_ratio = (roads.graph[tgtDesc]->pt.y() - hintLine[roads.graph[tgtDesc]->properties["group_id"].toInt()].y()) / D;
					//if (!G::getBool("fadeOut") || expf(-(SQR(x_ratio) + SQR(y_ratio)) / 2 / sigma2) >= Util::genRand(0, threshold)) {
					//if (!G::getBool("fadeOut") || expf(-sqrtf(SQR(x_ratio) + SQR(y_ratio))) >= Util::genRandNormal(expf(-1.2f), 0.1f)) {//Util::genRand(0, 0.65f)) {
					if ((expf(-(SQR(x_ratio) + SQR(y_ratio)) / 2 / sigma2) >= Util::genRand(threshold1 * 0.8f, threshold1)) && (f.area.contains(f.reducedRoads(roadType).graph[next_ex_v_desc]->pt))) {
						//f.reducedRoads(roadType).graph[next_ex_v_desc]->properties["used"] = true;
						roads.graph[tgtDesc]->properties["generation_type"] = "example";
						roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
						if (roadType == RoadEdge::TYPE_AVENUE) {
							roads.graph[tgtDesc]->properties["example_desc"] = next_ex_v_desc;
						} else {
							roads.graph[tgtDesc]->properties["example_street_desc"] = next_ex_v_desc;
						}
						roads.graph[tgtDesc]->properties["rotation_angle"] = roads.graph[srcDesc]->properties["rotation_angle"];
					}
				}
			} else {
				seeds.push_back(tgtDesc);
			}
		} else {
			// ターゲットエリアの外に出たら
			if (byExample) {
				roads.graph[tgtDesc]->properties["generation_type"] = "example";
				roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
				if (roadType == RoadEdge::TYPE_AVENUE) {
					roads.graph[tgtDesc]->properties["example_desc"] = next_ex_v_desc;
				} else {
					roads.graph[tgtDesc]->properties["example_street_desc"] = next_ex_v_desc;
				}
			} else {
				roads.graph[tgtDesc]->properties["generation_type"] = "mixed";
			}

			roads.graph[tgtDesc]->onBoundary = true;
		}

		// Example道路のDeadendは、Deadendとして正式に登録する
		if (byExample && GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) == 1) {
			roads.graph[tgtDesc]->properties["deadend"] = true;
		}
	}

	RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, srcDesc, tgtDesc, new_edge);

	// 新しいエッジにgroup_id、ex_idを引き継ぐ
	roads.graph[e_desc]->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
	roads.graph[e_desc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];

	// 新しいエッジのgeneration_typeを設定
	roads.graph[e_desc]->properties["generation_type"] = byExample ? "example" : "mixed";
	if (roads.graph[srcDesc]->properties["generation_type"] == "mixed") {
		roads.graph[e_desc]->properties["generation_type"] = "mixed";
	}
	*/

	return true;
}

/**
 * 指定されたpolylineに従って、srcDesc頂点からエッジを伸ばす。
 * エッジの端点が、srcDescとは違うセルに入る場合は、falseを返却する。
 */
bool PatchRoadGenerator::growRoadSegment(int roadType, RoadVertexDesc srcDesc, ExFeature& f, const Polyline2D &polyline, int lanes, RoadVertexDesc next_ex_v_desc, bool byExample, float snapFactor, float angleTolerance, std::list<RoadVertexDesc> &seeds) {
	float sigma2 = SQR(G::getFloat("interpolationSigma1"));
	float threshold1 = G::getFloat("interpolationThreshold1");

	bool intercepted = false;

	float z0 = vboRenderManager->getTerrainHeight(roads.graph[srcDesc]->pt.x(), roads.graph[srcDesc]->pt.y(), true);

	// 新しいエッジを生成
	RoadEdgePtr new_edge = RoadEdgePtr(new RoadEdge(roadType, lanes));
	QVector2D pt = roads.graph[srcDesc]->pt;
	for (int i = 0; i < polyline.size(); ++i) {
		if (i > 0) {
			pt += polyline[i] - polyline[i - 1];
		}
		//QVector2D pt = roads.graph[srcDesc]->pt + polyline[i];

		new_edge->polyline.push_back(pt);

		// 水没、または、山の上なら、道路生成をストップ
		float z = vboRenderManager->getTerrainHeight(pt.x(), pt.y(), true);
		if ((roadType == RoadEdge::TYPE_AVENUE && z < G::getFloat("seaLevelForAvenue")) || (roadType == RoadEdge::TYPE_STREET && z < G::getFloat("seaLevelForStreet"))) {
			// 最初っから水没している場合は、そもそもエッジ生成をキャンセル
			if (new_edge->polyline.size() <= 1) return false;

			RoadGeneratorHelper::bendEdgeBySteepElevationChange(roadType, new_edge->polyline, vboRenderManager);
			pt = new_edge->polyline.back();

			intercepted = true;
		}

		// if the slope is too steep, bend the direction to avoid climbing up
		if (z0 >= 70.0f && fabs(z - z0) > 7.0f) {
			RoadGeneratorHelper::bendEdgeBySteepElevationChange(new_edge->polyline, z0, vboRenderManager);
			pt = new_edge->polyline.back();
			z = vboRenderManager->getTerrainHeight(pt.x(), pt.y(), true);
			z0 = z;
		}

		// 他のエッジと交差したら、道路生成をストップ
		QVector2D intPoint;
		//if (roadType == RoadEdge::TYPE_STREET && GraphUtil::isIntersect(roads, new_edge->polyline, intPoint)) {
		if (GraphUtil::isIntersect(roads, new_edge->polyline, srcDesc, intPoint)) {
			new_edge->polyline.erase(new_edge->polyline.begin() + new_edge->polyline.size() - 1);
			new_edge->polyline.push_back(intPoint);

			intercepted = true;

			// エッジ長が最短thresholdより短い場合は、キャンセル
			if (new_edge->getLength() < 30.0f) return false;

			// エッジがgoingStraightではなく、且つ、もとのエッジ長の半分未満にinterceptされた場合は、キャンセル
			//if (!goingStraight && new_edge->getLength() < polyline.length() * 0.5f) return false;
		}

		if (intercepted) break;
	}

	if (new_edge->polyline.size() == 1) return false;

	RoadVertexDesc tgtDesc;

	if (byExample) {
		//snapFactor = 0.01f;
	}

	bool snapTried = false;
	bool snapDone = false;
	RoadEdgeDesc closestEdge;
	QVector2D intPoint;

	// snap to myself (loop edge)
	if ((new_edge->polyline.last() - roads.graph[srcDesc]->pt).lengthSquared() < 0.1f) {
		tgtDesc = srcDesc;
		snapTried = true;
		snapDone = true;
	}
	
	if (!snapDone && RoadGeneratorHelper::getCloseVertex(roads, new_edge->polyline.last(), byExample, roads.graph[srcDesc]->properties["group_id"].toInt(), (new_edge->polyline.back() - new_edge->polyline[0]).length() * snapFactor, srcDesc, tgtDesc)) {
		snapTried = true;

		if (byExample && roads.graph[tgtDesc]->properties["generation_type"] == "example" && roads.graph[tgtDesc]->properties["group_id"] == roads.graph[srcDesc]->properties["group_id"]) {
			angleTolerance = 0.01f;
		}

		// 他の頂点にスナップ
		Polyline2D new_polyline = new_edge->polyline;
		GraphUtil::movePolyline(roads, new_polyline, roads.graph[srcDesc]->pt, roads.graph[tgtDesc]->pt);

		if (RoadGeneratorHelper::submerged(roadType, new_polyline, vboRenderManager)) {
			new_polyline = new_edge->polyline;
			new_polyline.push_back(roads.graph[tgtDesc]->pt);
		}

		// 他のエッジと交差するか
		if (!GraphUtil::isIntersect(roads, new_polyline)) {
			std::reverse(new_polyline.begin(), new_polyline.end());
			if (!GraphUtil::hasRedundantEdge(roads, tgtDesc, new_polyline, angleTolerance)) {
				new_edge->polyline = new_polyline;
				snapDone = true;
			}
		}
	}
	
	if (!snapDone && RoadGeneratorHelper::getCloseEdge(roads, new_edge->polyline.last(), byExample, roads.graph[srcDesc]->properties["group_id"].toInt(), (new_edge->polyline.back() - new_edge->polyline[0]).length() * snapFactor, srcDesc, closestEdge, intPoint)) {
		snapTried = true;

		if (byExample && roads.graph[closestEdge]->properties["generation_type"] == "example" && roads.graph[closestEdge]->properties["group_id"] == roads.graph[srcDesc]->properties["group_id"]) {
			angleTolerance = 0.01f;
		}

		// 他のエッジにスナップ
		tgtDesc = GraphUtil::splitEdge(roads, closestEdge, intPoint);
		roads.graph[tgtDesc]->properties["generation_type"] = "snapped";
		roads.graph[tgtDesc]->properties["group_id"] = roads.graph[closestEdge]->properties["group_id"];
		roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[closestEdge]->properties["ex_id"];
		roads.graph[tgtDesc]->properties.remove("example_desc");

		GraphUtil::movePolyline(roads, new_edge->polyline, roads.graph[srcDesc]->pt, roads.graph[tgtDesc]->pt);
		if (!GraphUtil::isIntersect(roads, new_edge->polyline)) {
			if (!GraphUtil::hasRedundantEdge(roads, tgtDesc, new_edge->polyline, angleTolerance)) {
				snapDone = true;
				roads.graph[tgtDesc]->properties["parent"] = srcDesc;
			}
		}
	}

	if (snapTried && !snapDone) return false;

	if (!snapDone) {
		// 頂点を追加
		RoadVertexPtr v = RoadVertexPtr(new RoadVertex(new_edge->polyline.last()));
		tgtDesc = GraphUtil::addVertex(roads, v);
		roads.graph[tgtDesc]->properties["parent"] = srcDesc;

		// 新しい頂点にgroup_id、ex_idを引き継ぐ
		roads.graph[tgtDesc]->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
		roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];

		// 新しい頂点のgeneration_typeを設定
		roads.graph[tgtDesc]->properties["generation_type"] = byExample ? "example" : "mixed";

		if (targetArea.contains(new_edge->polyline.last())) {
			roads.graph[tgtDesc]->properties["generation_type"] = "mixed";

			// シードに追加する
			if (byExample && !intercepted) {
				//if (roadType == RoadEdge::TYPE_AVENUE || GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
				if (f.reducedRoads(roadType).graph[next_ex_v_desc]->onBoundary || GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
					seeds.push_back(tgtDesc);
				}

				// 対応するExampleが存在する場合は、それを設定する
				//if (!f.reducedRoads(roadType).graph[next_ex_v_desc]->properties.contains("used") && GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {
				if (GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) > 1) {

					// 初期シード位置から離れると、mixedに変わる確率が上がる
					float D = sqrtf(f.area.area());
					float x_ratio = (roads.graph[tgtDesc]->pt.x() - hintLine[roads.graph[tgtDesc]->properties["group_id"].toInt()].x()) / D;
					float y_ratio = (roads.graph[tgtDesc]->pt.y() - hintLine[roads.graph[tgtDesc]->properties["group_id"].toInt()].y()) / D;
					//if (!G::getBool("fadeOut") || expf(-(SQR(x_ratio) + SQR(y_ratio)) / 2 / sigma2) >= Util::genRand(0, threshold)) {
					//if (!G::getBool("fadeOut") || expf(-sqrtf(SQR(x_ratio) + SQR(y_ratio))) >= Util::genRandNormal(expf(-1.2f), 0.1f)) {//Util::genRand(0, 0.65f)) {
					if ((expf(-(SQR(x_ratio) + SQR(y_ratio)) / 2 / sigma2) >= Util::genRand(threshold1 * 0.8f, threshold1)) && (f.area.contains(f.reducedRoads(roadType).graph[next_ex_v_desc]->pt))) {
						//f.reducedRoads(roadType).graph[next_ex_v_desc]->properties["used"] = true;
						roads.graph[tgtDesc]->properties["generation_type"] = "example";
						roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
						if (roadType == RoadEdge::TYPE_AVENUE) {
							roads.graph[tgtDesc]->properties["example_desc"] = next_ex_v_desc;
						} else {
							roads.graph[tgtDesc]->properties["example_street_desc"] = next_ex_v_desc;
						}
						roads.graph[tgtDesc]->properties["rotation_angle"] = roads.graph[srcDesc]->properties["rotation_angle"];
					}
				}
			} else {
				seeds.push_back(tgtDesc);
			}
		} else {
			// ターゲットエリアの外に出たら
			if (byExample) {
				roads.graph[tgtDesc]->properties["generation_type"] = "example";
				roads.graph[tgtDesc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];
				if (roadType == RoadEdge::TYPE_AVENUE) {
					roads.graph[tgtDesc]->properties["example_desc"] = next_ex_v_desc;
				} else {
					roads.graph[tgtDesc]->properties["example_street_desc"] = next_ex_v_desc;
				}
			} else {
				roads.graph[tgtDesc]->properties["generation_type"] = "mixed";
			}

			roads.graph[tgtDesc]->onBoundary = true;
		}

		// Example道路のDeadendは、Deadendとして正式に登録する
		if (byExample && GraphUtil::getDegree(f.reducedRoads(roadType), next_ex_v_desc) == 1) {
			roads.graph[tgtDesc]->properties["deadend"] = true;
		}
	}

	RoadEdgeDesc e_desc = GraphUtil::addEdge(roads, srcDesc, tgtDesc, new_edge);

	// 新しいエッジにgroup_id、ex_idを引き継ぐ
	roads.graph[e_desc]->properties["group_id"] = roads.graph[srcDesc]->properties["group_id"];
	roads.graph[e_desc]->properties["ex_id"] = roads.graph[srcDesc]->properties["ex_id"];

	// 新しいエッジのgeneration_typeを設定
	roads.graph[e_desc]->properties["generation_type"] = byExample ? "example" : "mixed";
	if (roads.graph[srcDesc]->properties["generation_type"] == "mixed") {
		roads.graph[e_desc]->properties["generation_type"] = "mixed";
	}

	return true;
}

/**
 * PMに従って、カーネルを合成する
 * edgesに、エッジのリストを格納して返却する。
 * 各エッジのpolylineは、(0, 0)からスタートする。つまり、始点となる頂点の座標を(0, 0)とするということだ。
 */
void PatchRoadGenerator::synthesizeItem(int roadType, RoadVertexDesc v_desc, std::vector<RoadEdgePtr> &edges) {


	// 当該頂点から出るエッジをリストアップし、基底の方向を決定する
	float direction = 0.0f;
	{
		RoadOutEdgeIter ei, eend;
		for (boost::tie(ei, eend) = boost::out_edges(v_desc, roads.graph); ei != eend; ++ei) {
			if (!roads.graph[*ei]->valid) continue;

			Polyline2D polyline  = GraphUtil::orderPolyLine(roads, *ei, v_desc);
			QVector2D vec = polyline[1] - polyline[0];
			direction = atan2f(vec.y(), vec.x());
			break;
		}
	}

	for (int i = 0; i < 4; ++i) {
		RoadEdgePtr e = RoadEdgePtr(new RoadEdge(roadType, 1));
		e->polyline.push_back(QVector2D(0, 0));
		e->polyline.push_back(QVector2D(500.0f * cosf(direction), 500.0f * sinf(direction)));
		
		edges.push_back(e);

		direction += 3.141592653 * 0.5f;
	}
}

/**
 * エッジのジオミトリを、exampleに置換する
 */
void PatchRoadGenerator::replaceEdgeByExample(ExFeature &f, int roadType, RoadEdgePtr edge) {
	float length = edge->getLength();

	float min_diff = std::numeric_limits<float>::max();
	RoadEdgePtr e;

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(f.reducedRoads(roadType).graph); ei != eend; ++ei) {
		if (!f.reducedRoads(roadType).graph[*ei]->valid) continue;

		if (f.reducedRoads(roadType).graph[*ei]->properties.contains("shape_id")) continue;

		float length_diff = fabs(length - f.reducedRoads(roadType).graph[*ei]->getLength());
		if (length_diff < min_diff) {
			min_diff = length_diff;
			e = f.reducedRoads(roadType).graph[*ei];
		}
	}

	QVector2D offset = edge->polyline[0];
	QVector2D vec1 = edge->polyline.last() - edge->polyline[0];

	QVector2D vec2 = e->polyline.last() - e->polyline[0];
	float angle = Util::diffAngle(vec1, vec2, false);

	edge->polyline = e->polyline;
	edge->polyline.scale(length / edge->polyline.length());
	edge->polyline.translate(-edge->polyline[0] + offset);
	edge->polyline.rotate(-Util::rad2deg(angle), offset);

	edge->polyline = GraphUtil::finerEdge(edge->polyline, 10.0f);
}

bool PatchRoadGenerator::checkCrossing(RoadGraph& replacementGraph) {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(replacementGraph.graph); ei != eend; ++ei) {
		if (!replacementGraph.graph[*ei]->valid) continue;

		if (GraphUtil::isIntersect(roads, replacementGraph.graph[*ei]->polyline)) return true;
	}

	return false;
}

void PatchRoadGenerator::saveRoadImage(RoadGraph& roads, const char* filename) {
	BBox bbox = GraphUtil::getAABoundingBox(roads);
	cv::Mat img(bbox.dy() + 1, bbox.dx() + 1, CV_8UC3, cv::Scalar(0, 0, 0));

	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(roads.graph); ei != eend; ++ei) {
		if (!roads.graph[*ei]->valid) continue;

		for (int pl = 0; pl < roads.graph[*ei]->polyline.size() - 1; ++pl) {
			int x1 = roads.graph[*ei]->polyline[pl].x() - bbox.minPt.x();
			int y1 = img.rows - (roads.graph[*ei]->polyline[pl].y() - bbox.minPt.y());
			int x2 = roads.graph[*ei]->polyline[pl + 1].x() - bbox.minPt.x();
			int y2 = img.rows - (roads.graph[*ei]->polyline[pl + 1].y() - bbox.minPt.y());
			cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(128, 128, 128), 3);
		}
	}

	RoadVertexIter vi, vend;
	for (boost::tie(vi, vend) = boost::vertices(roads.graph); vi != vend; ++vi) {
		if (!roads.graph[*vi]->valid) continue;

		int x = roads.graph[*vi]->pt.x() - bbox.minPt.x();
		int y = img.rows - (roads.graph[*vi]->pt.y() - bbox.minPt.y());

		// onBoundaryを描画
		if (roads.graph[*vi]->onBoundary) {
			cv::circle(img, cv::Point(x, y), 10, cv::Scalar(0, 255, 255), 3);
		}

		// deadendを描画
		if (roads.graph[*vi]->deadend) {
			cv::circle(img, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), 3);
		}

		// 頂点IDを描画
		QString str = QString::number(*vi);
		cv::putText(img, str.toUtf8().data(), cv::Point(x, y), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
	}

	cv::imwrite(filename, img);
}

