#include "MainWindow.h"
#include <QFileDialog>
#include <QShortcut>
#include "TerrainSizeInputDialog.h"
#include "CircleHoughTransform.h"
#include "ShapeDetector.h"
#include "GraphUtil.h"
#include "CameraSequence.h"
#include "Util.h"
#include "CircleDetectioNScaleInputDialog.h"
#include "VideoCaptureSettingDialog.h"
#include "GrowingVideoSettingDialog.h"
#include "BlendingVideoSettingDialog.h"
#include "InterpolationVideoSettingDialog.h"
#include "RotationVideoSettingDialog.h"
#include "StructureDetectionSettingDialog.h"
#include "RoadGeneratorHelper.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	// setup the docking widgets
	controlWidget = new ControlWidget(this);
	propertyWidget = new PropertyWidget(this);

	// setup the toolbar
	ui.fileToolBar->addAction(ui.actionNewTerrain);
	ui.fileToolBar->addAction(ui.actionOpenTerrain);
	ui.areaToolBar->addAction(ui.actionAreaSelect);
	ui.areaToolBar->addAction(ui.actionAreaCreate);
	ui.areaToolBar->addAction(ui.actionHintLine);
	ui.areaToolBar->addAction(ui.actionAvenueSketch);

	ui.actionAreaSelect->setChecked(true);

	// register the menu's action handlers
	connect(ui.actionNewTerrain, SIGNAL(triggered()), this, SLOT(onNewTerrain()));
	connect(ui.actionOpenTerrain, SIGNAL(triggered()), this, SLOT(onLoadTerrain()));
	connect(ui.actionSaveTerrain, SIGNAL(triggered()), this, SLOT(onSaveTerrain()));
	connect(ui.actionLoadRoads, SIGNAL(triggered()), this, SLOT(onLoadRoads()));
	connect(ui.actionAddRoads, SIGNAL(triggered()), this, SLOT(onAddRoads()));
	connect(ui.actionSaveRoads, SIGNAL(triggered()), this, SLOT(onSaveRoads()));
	connect(ui.actionClearRoads, SIGNAL(triggered()), this, SLOT(onClearRoads()));
	connect(ui.actionLoadAreas, SIGNAL(triggered()), this, SLOT(onLoadAreas()));
	connect(ui.actionSaveAreas, SIGNAL(triggered()), this, SLOT(onSaveAreas()));
	connect(ui.actionSaveImage, SIGNAL(triggered()), this, SLOT(onSaveImage()));
	connect(ui.actionSaveImageHD, SIGNAL(triggered()), this, SLOT(onSaveImageHD()));
	connect(ui.actionLoadCamera, SIGNAL(triggered()), this, SLOT(onLoadCamera()));
	connect(ui.actionSaveCamera, SIGNAL(triggered()), this, SLOT(onSaveCamera()));
	connect(ui.actionResetCamera, SIGNAL(triggered()), this, SLOT(onResetCamera()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.menuArea, SIGNAL(aboutToShow()), this, SLOT(onAreaMenu()));
	connect(ui.actionAreaSelect, SIGNAL(triggered()), this, SLOT(onAreaSelect()));
	connect(ui.actionAreaCreate, SIGNAL(triggered()), this, SLOT(onAreaCreate()));
	connect(ui.actionAreaDelete, SIGNAL(triggered()), this, SLOT(onAreaDelete()));
	connect(ui.actionAreaDeleteAll, SIGNAL(triggered()), this, SLOT(onAreaDeleteAll()));
	connect(ui.actionHintLine, SIGNAL(triggered()), this, SLOT(onHintLine()));
	connect(ui.actionCutRoads, SIGNAL(triggered()), this, SLOT(onCutRoads()));
	connect(ui.actionHighwaySketch, SIGNAL(triggered()), this, SLOT(onHighwaySketch()));
	connect(ui.actionAvenueSketch, SIGNAL(triggered()), this, SLOT(onAvenueSketch()));
	connect(ui.actionStreetSketch, SIGNAL(triggered()), this, SLOT(onStreetSketch()));
	connect(ui.actionControlPoints, SIGNAL(triggered()), this, SLOT(onControlPoints()));
	connect(ui.actionDetectCircle, SIGNAL(triggered()), this, SLOT(onDetectCircle()));
	connect(ui.actionDetectStructure, SIGNAL(triggered()), this, SLOT(onDetectStructure()));	
	connect(ui.actionConvert, SIGNAL(triggered()), this, SLOT(onConvert()));	
	connect(ui.actionGenerate2D, SIGNAL(triggered()), this, SLOT(onGenerate2D()));
	connect(ui.actionGenerate3D, SIGNAL(triggered()), this, SLOT(onGenerate3D()));
	connect(ui.actionGenerate3DRoads, SIGNAL(triggered()), this, SLOT(onGenerate3DRoads()));
	connect(ui.actionDisplayHighway, SIGNAL(triggered()), this, SLOT(onDisplayRoads()));
	connect(ui.actionDisplayBoulevard, SIGNAL(triggered()), this, SLOT(onDisplayRoads()));
	connect(ui.actionRenderingDefault, SIGNAL(triggered()), this, SLOT(onRenderingDefault()));
	connect(ui.actionRenderingTexture, SIGNAL(triggered()), this, SLOT(onRenderingTexture()));
	connect(ui.actionRenderingGroupBy, SIGNAL(triggered()), this, SLOT(onRenderingGroupBy()));
	connect(ui.actionRenderingGenerationType, SIGNAL(triggered()), this, SLOT(onRenderingGenerationType()));
	connect(ui.actionGenerateRegularGrid, SIGNAL(triggered()), this, SLOT(onGenerateRegularGrid()));
	connect(ui.actionGenerateCurvyGrid, SIGNAL(triggered()), this, SLOT(onGenerateCurvyGrid()));
	connect(ui.actionRotationVideo, SIGNAL(triggered()), this, SLOT(onRotationVideo()));
	connect(ui.actionInterpolationVideo, SIGNAL(triggered()), this, SLOT(onInterpolationVideo()));
	connect(ui.actionBlendingVideo, SIGNAL(triggered()), this, SLOT(onBlendingVideo()));
	connect(ui.actionGrowingVideo, SIGNAL(triggered()), this, SLOT(onGrowingVideo()));
	connect(ui.action3DVideo, SIGNAL(triggered()), this, SLOT(on3DVideo()));
	connect(ui.actionMountainVideo, SIGNAL(triggered()), this, SLOT(onMountainVideo()));
	connect(ui.actionTerrainGeneration, SIGNAL(triggered()), this, SLOT(onTerrainGeneration()));
	connect(ui.actionUpdateMountain, SIGNAL(triggered()), this, SLOT(onUpdateMountain()));
	connect(ui.actionTerrainSegmentation, SIGNAL(triggered()), this, SLOT(onTerrainSegmentation()));
	connect(ui.actionControlWidget, SIGNAL(triggered()), this, SLOT(onShowControlWidget()));
	connect(ui.actionPropertyWidget, SIGNAL(triggered()), this, SLOT(onShowPropertyWidget()));
	connect(ui.actionDebug, SIGNAL(triggered()), this, SLOT(onDebug()));

	// setup the GL widget
	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);

	controlWidget->show();
	addDockWidget(Qt::LeftDockWidgetArea, controlWidget);

	urbanGeometry = new UrbanGeometry(this);

	mode = MODE_AREA_SELECT;
	//imgCount=0;
}

MainWindow::~MainWindow() {
}

void MainWindow::keyPressEvent(QKeyEvent* e) {
	glWidget->keyPressEvent(e);
}

void MainWindow::keyReleaseEvent(QKeyEvent* e) {
	glWidget->keyReleaseEvent(e);
}

void MainWindow::onNewTerrain() {
	TerrainSizeInputDialog dlg(this);
	if (dlg.exec() == QDialog::Accepted) {
		//urbanGeometry->newTerrain(dlg.width, dlg.depth, dlg.cellLength);
		glWidget->vboRenderManager.changeTerrainDimensions(dlg.side,dlg.cellResolution);
		glWidget->updateGL();
	}
}

void MainWindow::onLoadTerrain() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Load Terrain file..."), "", tr("Terrain Files (*.png *.jpg)"));
	if (filename.isEmpty()) return;
	glWidget->vboRenderManager.vboTerrain.loadTerrain(filename);
	//urbanGeometry->loadTerrain(filename);

	glWidget->updateGL();
}

void MainWindow::onSaveTerrain() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Terrain file..."), "", tr("Terrain Files (*.png)"));
	if (filename.isEmpty()) return;
	glWidget->vboRenderManager.vboTerrain.saveTerrain(filename);
	//urbanGeometry->saveTerrain(filename);
}

void MainWindow::onLoadRoads() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Street Map file..."), "", tr("StreetMap Files (*.gsm)"));
	if (filename.isEmpty()) return;

	urbanGeometry->loadRoads(filename);

	glWidget->updateGL();
}

void MainWindow::onAddRoads() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Street Map file..."), "", tr("StreetMap Files (*.gsm)"));
	if (filename.isEmpty()) return;

	urbanGeometry->addRoads(filename);

	glWidget->updateGL();
}

void MainWindow::onSaveRoads() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Street Map file..."), "", tr("StreetMap Files (*.gsm)"));
	if (filename.isEmpty()) return;

	QApplication::setOverrideCursor(Qt::WaitCursor);

	urbanGeometry->saveRoads(filename);

	QApplication::restoreOverrideCursor();
}

void MainWindow::onClearRoads() {
	urbanGeometry->clearRoads();
	glWidget->updateGL();
}

void MainWindow::onLoadAreas() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Area file..."), "", tr("Area Files (*.area)"));
	if (filename.isEmpty()) return;
	
	urbanGeometry->loadAreas(filename);

	glWidget->updateGL();
}

void MainWindow::onSaveAreas() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Area Map file..."), "", tr("Area Files (*.area)"));
	if (filename.isEmpty()) return;

	QApplication::setOverrideCursor(Qt::WaitCursor);

	urbanGeometry->saveAreas(filename);

	QApplication::restoreOverrideCursor();
}

void MainWindow::onSaveImage() {
	if(QDir("screenshots").exists()==false) QDir().mkdir("screenshots");
	QString fileName="screenshots/"+QDate::currentDate().toString("yyMMdd")+"_"+QTime::currentTime().toString("HHmmss")+".png";
	glWidget->grabFrameBuffer().save(fileName);
	printf("Save %s\n",fileName.toAscii().constData());
}//

void MainWindow::onSaveImageHD() {
	if(QDir("screenshots").exists()==false) QDir().mkdir("screenshots");
	QString fileName="screenshots/"+QDate::currentDate().toString("yyMMdd")+"_"+QTime::currentTime().toString("HHmmss")+".png";
	glWidget->grabFrameBuffer().save(fileName);

	QString fileNameHD="screenshots/"+QDate::currentDate().toString("yyMMdd")+"_"+QTime::currentTime().toString("HHmmss")+"_HD.png";
	int cH=glWidget->height();
	int cW=glWidget->width();
	glWidget->resize(cW*3,cH*3);
	glWidget->updateGL();
	glWidget->grabFrameBuffer().save(fileNameHD);
	glWidget->resize(cW,cH);
	glWidget->updateGL();

	printf("Save HD %s\n",fileName.toAscii().constData());
}//

void MainWindow::onLoadCamera() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Camera file..."), "", tr("Area Files (*.cam)"));
	if (filename.isEmpty()) return;

	glWidget->camera->loadCameraPose(filename);
	glWidget->updateCamera();

	glWidget->updateGL();
}

void MainWindow::onSaveCamera() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Camera file..."), "", tr("Area Files (*.cam)"));
	if (filename.isEmpty()) return;
	
	glWidget->camera->saveCameraPose(filename);
}

void MainWindow::onResetCamera() {
	glWidget->camera->resetCamera();
	glWidget->updateCamera();
	glWidget->updateGL();
}

void MainWindow::onAreaMenu() {
	ui.actionAreaSelect->setChecked(mode == MODE_AREA_SELECT);
	ui.actionAreaCreate->setChecked(mode == MODE_AREA_CREATE);
	ui.actionHintLine->setChecked(mode == MODE_HINT_LINE);
	ui.actionHighwaySketch->setChecked(mode == MODE_HIGHWAY_SKETCH);
	ui.actionAvenueSketch->setChecked(mode == MODE_AVENUE_SKETCH);
	ui.actionStreetSketch->setChecked(mode == MODE_STREET_SKETCH);
	ui.actionControlPoints->setChecked(mode == MODE_CONTROL_POINTS);
}

void MainWindow::onAreaSelect() {
	mode = MODE_AREA_SELECT;
	ui.actionAreaCreate->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onAreaCreate() {
	mode = MODE_AREA_CREATE;
	ui.actionAreaSelect->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onAreaDelete() {
	urbanGeometry->areas.deleteArea();

	glWidget->updateGL();
}

void MainWindow::onAreaDeleteAll() {
	urbanGeometry->areas.clear();

	glWidget->updateGL();
}

void MainWindow::onHintLine() {
	mode = MODE_HINT_LINE;
	ui.actionAreaCreate->setChecked(false);
	ui.actionAreaSelect->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onCutRoads() {
	urbanGeometry->cutRoads();

	glWidget->updateGL();
}

void MainWindow::onHighwaySketch() {
	mode = MODE_HIGHWAY_SKETCH;
	ui.actionAreaCreate->setChecked(false);
	ui.actionAreaSelect->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onAvenueSketch() {
	mode = MODE_AVENUE_SKETCH;
	ui.actionAreaCreate->setChecked(false);
	ui.actionAreaSelect->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onStreetSketch() {
	mode = MODE_STREET_SKETCH;
	ui.actionAreaCreate->setChecked(false);
	ui.actionAreaSelect->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionControlPoints->setChecked(false);
}

void MainWindow::onControlPoints() {
	mode = MODE_CONTROL_POINTS;
	ui.actionAreaCreate->setChecked(false);
	ui.actionAreaSelect->setChecked(false);
	ui.actionHintLine->setChecked(false);
	ui.actionHighwaySketch->setChecked(false);
	ui.actionAvenueSketch->setChecked(false);
	ui.actionStreetSketch->setChecked(false);
}

void MainWindow::onDetectCircle() {
	CircleDetectionScaleInputDialog dlg(this);
	if (dlg.exec() == QDialog::Accepted) {
		CircleHoughTransform ht;
		urbanGeometry->shapes = ht.detect(urbanGeometry->roads, dlg.scale, 50.0f);
		glWidget->updateGL();
	}
}

void MainWindow::onDetectStructure() {
	StructureDetectionSettingDialog dlg(this);
	if (dlg.exec() == QDialog::Accepted) {
		urbanGeometry->shapes = ShapeDetector::detect(urbanGeometry->roads, dlg.scale, dlg.distance);

		std::vector<Patch> patches;
		patches = RoadGeneratorHelper::convertToPatch(RoadEdge::TYPE_AVENUE, urbanGeometry->roads, urbanGeometry->roads, urbanGeometry->shapes);

		// save patch images
		ExFeature::savePatchImages(RoadEdge::TYPE_AVENUE, 0, urbanGeometry->roads, patches, 10.0f, false);

		glWidget->updateGL();
	}
}

/*
void MainWindow::onConvert() {
	RoadEdgeIter ei, eend;
	for (boost::tie(ei, eend) = boost::edges(urbanGeometry->roads.graph); ei != eend; ++ei) {
		if (urbanGeometry->roads.graph[*ei]->type == RoadEdge::TYPE_BOULEVARD) {
			urbanGeometry->roads.graph[*ei]->type = RoadEdge::TYPE_AVENUE;
		}
	}
}
*/

void MainWindow::onGenerate3D() {
	glWidget->generate3DGeometry();
	glWidget->updateGL();
}

void MainWindow::onGenerate3DRoads() {
	glWidget->generate3DGeometry(true);//true just roads
	glWidget->updateGL();
}//

void MainWindow::onGenerate2D() {
	glWidget->generate2DGeometry();
	glWidget->updateGL();
}

void MainWindow::onDisplayRoads() {
	urbanGeometry->roads.showHighways = ui.actionDisplayHighway->isChecked();
	urbanGeometry->roads.showBoulevards = ui.actionDisplayBoulevard->isChecked();
	urbanGeometry->roads.showAvenues = ui.actionDisplayAvenue->isChecked();
	urbanGeometry->roads.showLocalStreets = ui.actionDisplayLocalStreet->isChecked();
	urbanGeometry->roads.setModified();

	glWidget->updateGL();
}

void MainWindow::onRenderingDefault() {
	ui.actionRenderingTexture->setChecked(false);
	ui.actionRenderingGroupBy->setChecked(false);
	ui.actionRenderingGenerationType->setChecked(false);

	urbanGeometry->roads.renderMode = RoadGraph::RENDER_DEFAULT;
	urbanGeometry->roads.setModified();

	glWidget->updateGL();
}

void MainWindow::onRenderingTexture() {
	ui.actionRenderingDefault->setChecked(false);
	ui.actionRenderingGroupBy->setChecked(false);
	ui.actionRenderingGenerationType->setChecked(false);

	urbanGeometry->roads.renderMode = RoadGraph::RENDER_TEXTURE;
	urbanGeometry->roads.setModified();

	glWidget->updateGL();
}

void MainWindow::onRenderingGroupBy() {
	ui.actionRenderingDefault->setChecked(false);
	ui.actionRenderingTexture->setChecked(false);
	ui.actionRenderingGenerationType->setChecked(false);

	urbanGeometry->roads.renderMode = RoadGraph::RENDER_GROUPBY;
	urbanGeometry->roads.setModified();
	for (int i = 0; i < urbanGeometry->areas.size(); ++i) {
		urbanGeometry->areas[i]->roads.renderMode = RoadGraph::RENDER_GROUPBY;
		urbanGeometry->areas[i]->roads.setModified();
	}

	glWidget->updateGL();
}

void MainWindow::onRenderingGenerationType() {
	ui.actionRenderingDefault->setChecked(false);
	ui.actionRenderingTexture->setChecked(false);
	ui.actionRenderingGroupBy->setChecked(false);

	urbanGeometry->roads.renderMode = RoadGraph::RENDER_GENERATION_TYPE;
	urbanGeometry->roads.setModified();

	glWidget->updateGL();
}

void MainWindow::onGenerateRegularGrid() {
	GraphUtil::generateRegularGrid(urbanGeometry->roads, 5000, 500, 100);
	urbanGeometry->adaptToTerrain();
}

void MainWindow::onGenerateCurvyGrid() {
	GraphUtil::generateCurvyGrid(urbanGeometry->roads, 5000, 500, 100);
	urbanGeometry->adaptToTerrain();
}

void MainWindow::onRotationVideo() {
	if (urbanGeometry->areas.selectedIndex == -1) return;

	RotationVideoSettingDialog dlg(this);
	if (dlg.exec() != QDialog::Accepted) return;

	G::global()["numAvenueIterations"] = controlWidget->ui.lineEditNumAvenueIterations->text().toInt();
	G::global()["numStreetIterations"] = controlWidget->ui.lineEditNumStreetIterations->text().toInt();
	int numExamples = controlWidget->ui.lineEditNumExamples->text().toInt();
	G::global()["cleanAvenues"] = controlWidget->ui.checkBoxCleanAvenues->isChecked();
	G::global()["cleanStreets"] = controlWidget->ui.checkBoxCleanStreets->isChecked();
	G::global()["generateLocalStreets"] = controlWidget->ui.checkBoxLocalStreets->isChecked();
	G::global()["cropping"] = controlWidget->ui.checkBoxCropping->isChecked();
	G::global()["useLayer"] = controlWidget->ui.checkBoxUseLayer->isChecked();
	G::global()["removeSmallBlocks"] = controlWidget->ui.checkBoxRemoveSmallBlocks->isChecked();
	G::global()["minBlockSize"] = controlWidget->ui.lineEditMinBlockSize->text().toFloat();

	G::global()["houghScale"] = controlWidget->ui.lineEditHoughScale->text().toFloat();
	G::global()["avenuePatchDistance"] = controlWidget->ui.lineEditPatchDistance1->text().toFloat();
	G::global()["streetPatchDistance"] = controlWidget->ui.lineEditPatchDistance2->text().toFloat();
	G::global()["interpolationSigma1"] = controlWidget->ui.lineEditInterpolateSigma1->text().toFloat();
	G::global()["interpolationSigma2"] = controlWidget->ui.lineEditInterpolateSigma2->text().toFloat();
	G::global()["interpolationThreshold1"] = controlWidget->ui.lineEditInterpolateThreshold1->text().toFloat();
	G::global()["roadSnapFactor"] = controlWidget->ui.lineEditRoadSnapFactor->text().toFloat();
	G::global()["roadAngleTolerance"] = controlWidget->ui.lineEditRoadAngleTolerance->text().toFloat() / 180.0f * M_PI;
	G::global()["rotationForSteepSlope"] = controlWidget->ui.lineEditRotationForSteepSlope->text().toFloat() / 180.0f * M_PI;

	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	// backup the roads and area
	RoadGraph origRoads;
	GraphUtil::copyRoads(urbanGeometry->roads, origRoads);
	RoadArea area(*urbanGeometry->areas.selectedArea());

	// remove the area temporarily
	urbanGeometry->areas.deleteArea();

	std::vector<ExFeature> features;
	features.resize(numExamples);
	for (int i = 0; i < numExamples; ++i) {
		QString filename = QFileDialog::getOpenFileName(this, tr("Open Feature file..."), "", tr("StreetMap Files (*.xml)"));
		if (filename.isEmpty()) return;
	
		features[i].load(filename, false);
		//features[i].detectAvenueShapes(G::getFloat("avenuePatchDistance"));
		//features[i].detectStreetShapes(G::getFloat("streetPatchDistance"));
	}

	// every time when rotating by 1 degree, generate roads and capture the image
	int frame_no = 0;
	for (float theta = dlg.start / 180.0f * M_PI; theta <= dlg.end / 180.0f * M_PI; theta += dlg.step / 180.0f * M_PI) {
		G::global()["rotationAngle"] = theta;

		urbanGeometry->clear();

		// recover the original roads
		GraphUtil::copyRoads(origRoads, urbanGeometry->roads);

		// add area
		urbanGeometry->areas.add(RoadAreaPtr(new RoadArea(area)));
		urbanGeometry->areas.selectedIndex = urbanGeometry->areas.size() - 1;

		// cut the roads within the area
		GraphUtil::subtractRoads(urbanGeometry->roads, area.area, false);

		// generate roads within the area
		urbanGeometry->generateRoadsPatchMulti(features);
	
		urbanGeometry->areas.deleteArea();

		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/%1.png").arg(frame_no++, 4, 10, QChar('0'));
		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}
}

void MainWindow::onInterpolationVideo() {
	if (urbanGeometry->areas.selectedIndex == -1) return;

	InterpolationVideoSettingDialog dlg(this);
	if (dlg.exec() != QDialog::Accepted) return;
	
	G::global()["numAvenueIterations"] = controlWidget->ui.lineEditNumAvenueIterations->text().toInt();
	G::global()["numStreetIterations"] = controlWidget->ui.lineEditNumStreetIterations->text().toInt();
	int numExamples = controlWidget->ui.lineEditNumExamples->text().toInt();
	G::global()["cleanAvenues"] = controlWidget->ui.checkBoxCleanAvenues->isChecked();
	G::global()["cleanStreets"] = controlWidget->ui.checkBoxCleanStreets->isChecked();
	G::global()["generateLocalStreets"] = controlWidget->ui.checkBoxLocalStreets->isChecked();
	G::global()["cropping"] = controlWidget->ui.checkBoxCropping->isChecked();
	G::global()["useLayer"] = controlWidget->ui.checkBoxUseLayer->isChecked();
	G::global()["removeSmallBlocks"] = controlWidget->ui.checkBoxRemoveSmallBlocks->isChecked();
	G::global()["minBlockSize"] = controlWidget->ui.lineEditMinBlockSize->text().toFloat();

	G::global()["houghScale"] = controlWidget->ui.lineEditHoughScale->text().toFloat();
	G::global()["avenuePatchDistance"] = controlWidget->ui.lineEditPatchDistance1->text().toFloat();
	G::global()["streetPatchDistance"] = controlWidget->ui.lineEditPatchDistance2->text().toFloat();
	G::global()["interpolationThreshold1"] = controlWidget->ui.lineEditInterpolateThreshold1->text().toFloat();
	G::global()["rotationAngle"] = controlWidget->ui.lineEditRotationAngle->text().toFloat() / 180.0f * M_PI;
	G::global()["roadSnapFactor"] = controlWidget->ui.lineEditRoadSnapFactor->text().toFloat();
	G::global()["roadAngleTolerance"] = controlWidget->ui.lineEditRoadAngleTolerance->text().toFloat() / 180.0f * M_PI;
	G::global()["rotationForSteepSlope"] = controlWidget->ui.lineEditRotationForSteepSlope->text().toFloat() / 180.0f * M_PI;

	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	// backup the roads and area
	RoadGraph origRoads;
	GraphUtil::copyRoads(urbanGeometry->roads, origRoads);
	RoadArea area(*urbanGeometry->areas.selectedArea());

	// remove the area temporarily
	urbanGeometry->areas.deleteArea();

	std::vector<ExFeature> features;
	features.resize(numExamples);
	for (int i = 0; i < numExamples; ++i) {
		QString filename = QFileDialog::getOpenFileName(this, tr("Open Feature file..."), "", tr("StreetMap Files (*.xml)"));
		if (filename.isEmpty()) return;
	
		features[i].load(filename, true);
	}

	// every time when rotating by 1 degree, generate roads and capture the image
	int frame_no = 0;
	for (float s = dlg.start; s <= dlg.end; s += dlg.step) {
		float t = 1.0f - s;

		G::global()["interpolationSigma1"] = s;
		G::global()["interpolationSigma2"] = t;

		urbanGeometry->clear();

		// recover the original roads
		GraphUtil::copyRoads(origRoads, urbanGeometry->roads);

		// add area
		urbanGeometry->areas.add(RoadAreaPtr(new RoadArea(area)));
		urbanGeometry->areas.selectedIndex = urbanGeometry->areas.size() - 1;

		// cut the roads within the area
		GraphUtil::subtractRoads(urbanGeometry->roads, area.area, false);

		// generate roads within the area
		urbanGeometry->generateRoadsPM(features);
	
		urbanGeometry->areas.deleteArea();

		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/%1.png").arg(frame_no++, 4, 10, QChar('0'));
		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}
}

void MainWindow::onBlendingVideo() {
	if (urbanGeometry->areas.selectedIndex == -1) return;

	BlendingVideoSettingDialog dlg(this);
	if (dlg.exec() != QDialog::Accepted) return;

	G::global()["numAvenueIterations"] = controlWidget->ui.lineEditNumAvenueIterations->text().toInt();
	G::global()["numStreetIterations"] = controlWidget->ui.lineEditNumStreetIterations->text().toInt();
	int numExamples = controlWidget->ui.lineEditNumExamples->text().toInt();
	G::global()["cleanAvenues"] = controlWidget->ui.checkBoxCleanAvenues->isChecked();
	G::global()["cleanStreets"] = controlWidget->ui.checkBoxCleanStreets->isChecked();
	G::global()["generateLocalStreets"] = controlWidget->ui.checkBoxLocalStreets->isChecked();
	G::global()["cropping"] = controlWidget->ui.checkBoxCropping->isChecked();
	G::global()["useLayer"] = controlWidget->ui.checkBoxUseLayer->isChecked();
	G::global()["removeSmallBlocks"] = controlWidget->ui.checkBoxRemoveSmallBlocks->isChecked();
	G::global()["minBlockSize"] = controlWidget->ui.lineEditMinBlockSize->text().toFloat();

	G::global()["houghScale"] = controlWidget->ui.lineEditHoughScale->text().toFloat();
	G::global()["avenuePatchDistance"] = controlWidget->ui.lineEditPatchDistance1->text().toFloat();
	G::global()["streetPatchDistance"] = controlWidget->ui.lineEditPatchDistance2->text().toFloat();
	G::global()["interpolationSigma1"] = controlWidget->ui.lineEditInterpolateSigma1->text().toFloat();
	G::global()["interpolationSigma2"] = controlWidget->ui.lineEditInterpolateSigma2->text().toFloat();
	G::global()["interpolationThreshold1"] = controlWidget->ui.lineEditInterpolateThreshold1->text().toFloat();
	G::global()["rotationAngle"] = controlWidget->ui.lineEditRotationAngle->text().toFloat() / 180.0f * M_PI;
	G::global()["roadSnapFactor"] = controlWidget->ui.lineEditRoadSnapFactor->text().toFloat();
	G::global()["roadAngleTolerance"] = controlWidget->ui.lineEditRoadAngleTolerance->text().toFloat() / 180.0f * M_PI;
	G::global()["rotationForSteepSlope"] = controlWidget->ui.lineEditRotationForSteepSlope->text().toFloat() / 180.0f * M_PI;

	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	// backup the roads and area
	RoadGraph origRoads;
	GraphUtil::copyRoads(urbanGeometry->roads, origRoads);
	RoadArea area(*urbanGeometry->areas.selectedArea());

	// remove the area temporarily
	urbanGeometry->areas.deleteArea();

	std::vector<ExFeature> features;
	features.resize(numExamples);
	for (int i = 0; i < numExamples; ++i) {
		QString filename = QFileDialog::getOpenFileName(this, tr("Open Feature file..."), "", tr("StreetMap Files (*.xml)"));
		if (filename.isEmpty()) return;
	
		features[i].load(filename, false);
	}

	// every time when rotating by 1 degree, generate roads and capture the image
	int frame_no = 0;
	for (float s = dlg.start; s <= dlg.end; s += dlg.step) {
		float t = 1.0f - s;
	//for (int i = 0; i <= 100; i += 10) {
		//float s = (float)(100 - i) / 100.0f;
		//float t = (float)i / 100.0f;

		G::global()["interpolationSigma1"] = s;
		G::global()["interpolationSigma2"] = t;

		urbanGeometry->clear();

		// recover the original roads
		GraphUtil::copyRoads(origRoads, urbanGeometry->roads);

		// add area
		urbanGeometry->areas.add(RoadAreaPtr(new RoadArea(area)));
		urbanGeometry->areas.selectedIndex = urbanGeometry->areas.size() - 1;

		// cut the roads within the area
		GraphUtil::subtractRoads(urbanGeometry->roads, area.area, false);

		// generate roads within the area
		urbanGeometry->generateRoadsPatchMulti(features);
	
		urbanGeometry->areas.deleteArea();

		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/%1.png").arg(frame_no++, 4, 10, QChar('0'));
		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}
}

void MainWindow::onGrowingVideo() {
	if (urbanGeometry->areas.selectedIndex == -1) return;

	GrowingVideoSettingDialog dlg(this);
	if (dlg.exec() != QDialog::Accepted) return;

	int numExamples = controlWidget->ui.lineEditNumExamples->text().toInt();
	G::global()["cleanAvenues"] = controlWidget->ui.checkBoxCleanAvenues->isChecked();
	G::global()["cleanStreets"] = controlWidget->ui.checkBoxCleanStreets->isChecked();
	G::global()["cropping"] = controlWidget->ui.checkBoxCropping->isChecked();
	G::global()["useLayer"] = controlWidget->ui.checkBoxUseLayer->isChecked();
	G::global()["removeSmallBlocks"] = controlWidget->ui.checkBoxRemoveSmallBlocks->isChecked();
	G::global()["minBlockSize"] = controlWidget->ui.lineEditMinBlockSize->text().toFloat();

	G::global()["houghScale"] = controlWidget->ui.lineEditHoughScale->text().toFloat();
	G::global()["avenuePatchDistance"] = controlWidget->ui.lineEditPatchDistance1->text().toFloat();
	G::global()["streetPatchDistance"] = controlWidget->ui.lineEditPatchDistance2->text().toFloat();
	G::global()["interpolationSigma1"] = controlWidget->ui.lineEditInterpolateSigma1->text().toFloat();
	G::global()["interpolationSigma2"] = controlWidget->ui.lineEditInterpolateSigma2->text().toFloat();
	G::global()["interpolationThreshold1"] = controlWidget->ui.lineEditInterpolateThreshold1->text().toFloat();
	G::global()["rotationAngle"] = controlWidget->ui.lineEditRotationAngle->text().toFloat() / 180.0f * M_PI;
	G::global()["roadSnapFactor"] = controlWidget->ui.lineEditRoadSnapFactor->text().toFloat();
	G::global()["roadAngleTolerance"] = controlWidget->ui.lineEditRoadAngleTolerance->text().toFloat() / 180.0f * M_PI;
	G::global()["rotationForSteepSlope"] = controlWidget->ui.lineEditRotationForSteepSlope->text().toFloat() / 180.0f * M_PI;

	// backup the roads and area
	RoadGraph origRoads;
	GraphUtil::copyRoads(urbanGeometry->roads, origRoads);
	RoadArea area(*urbanGeometry->areas.selectedArea());

	// remove the area temporarily
	urbanGeometry->areas.deleteArea();

	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	std::vector<ExFeature> features;
	features.resize(numExamples);
	for (int i = 0; i < numExamples; ++i) {
		QString filename = QFileDialog::getOpenFileName(this, tr("Open Feature file..."), "", tr("StreetMap Files (*.xml)"));
		if (filename.isEmpty()) return;
	
		if (dlg.generationMethod == "Multi Examples") {
			features[i].load(filename, false);
		} else if (dlg.generationMethod == "Warp") {
			features[i].load(filename, false);
		} else {
			features[i].load(filename, true);
		}
	}

	int frame_no = 0;
	for (int i = 0; i < dlg.numAvenueIterations; i+=dlg.frequency) {
		G::global()["numAvenueIterations"] = i;
		G::global()["numStreetIterations"] = 0;
		G::global()["generateLocalStreets"] = false;

		urbanGeometry->clear();

		// recover the original roads
		GraphUtil::copyRoads(origRoads, urbanGeometry->roads);

		// add area
		urbanGeometry->areas.add(RoadAreaPtr(new RoadArea(area)));
		urbanGeometry->areas.selectedIndex = urbanGeometry->areas.size() - 1;

		// cut the roads within the area
		GraphUtil::subtractRoads(urbanGeometry->roads, area.area, false);

		if (dlg.generationMethod == "Multi Examples") {
			urbanGeometry->generateRoadsPatchMulti(features);
		} else if (dlg.generationMethod == "Warp") {
			urbanGeometry->generateRoadsPatchWarp2(features);
		} else {
			urbanGeometry->generateRoadsPM(features);
		}

		urbanGeometry->areas.deleteArea();

		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/%1.png").arg(frame_no++, 4, 10, QChar('0'));
		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}

	for (int i = 0; i < dlg.numStreetIterations; i+=dlg.frequency) {
		G::global()["numAvenueIterations"] = controlWidget->ui.lineEditNumAvenueIterations->text().toInt();
		G::global()["numStreetIterations"] = i;
		G::global()["generateLocalStreets"] = true;

		urbanGeometry->clear();

		// recover the original roads
		GraphUtil::copyRoads(origRoads, urbanGeometry->roads);

		// add area
		urbanGeometry->areas.add(RoadAreaPtr(new RoadArea(area)));
		urbanGeometry->areas.selectedIndex = urbanGeometry->areas.size() - 1;

		// cut the roads within the area
		GraphUtil::subtractRoads(urbanGeometry->roads, area.area, false);

		if (dlg.generationMethod == "Multi Examples") {
			urbanGeometry->generateRoadsPatchMulti(features);
		} else if (dlg.generationMethod == "Warp") {
			urbanGeometry->generateRoadsPatchWarp2(features);
		} else {
			urbanGeometry->generateRoadsPM(features);
		}

		urbanGeometry->areas.deleteArea();

		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/%1.png").arg(frame_no++, 4, 10, QChar('0'));
		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}
}

void MainWindow::on3DVideo() {
	VideoCaptureSettingDialog dlg(this);
	if (dlg.exec() != QDialog::Accepted) return;

	QString filename = QFileDialog::getOpenFileName(this, tr("Open camera sequence file..."), "", tr("Camera Sequence Files (*.xml)"));
	if (filename.isEmpty()) return;
	
	CameraSequence cs;
	cs.load(filename, dlg.cameraStep);

	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	for (int i = 0; i < cs.cameras.size(); ++i) {
		glWidget->flyCamera = cs.cameras[i];
		glWidget->updateCamera();
		glWidget->updateGL();

		// save the framebuffer
		QString fileName = QString("video/fly%1.png").arg(i, 4, 10, QChar('0'));

		if (dlg.useHDImage) {
			int cH=glWidget->height();
			int cW=glWidget->width();
			glWidget->resize(cW*3,cH*3);
			glWidget->updateGL();
			glWidget->grabFrameBuffer().save(fileName);
			glWidget->resize(cW,cH);
			glWidget->updateGL();
		} else {
			glWidget->grabFrameBuffer().save(fileName);
		}
	}
}

void MainWindow::onMountainVideo() {
	// create the directory
	if(QDir("video").exists()==false) QDir().mkdir("video");

	for (int i = 0; i < 100; ++i) {
		onUpdateMountain();

		// save the framebuffer
		QString fileName = QString("video/mountain%1.png").arg(i, 4, 10, QChar('0'));
		//glWidget->grabFrameBuffer().save(fileName);

		int cH=glWidget->height();
		int cW=glWidget->width();
		glWidget->resize(cW*3,cH*3);
		glWidget->updateGL();
		glWidget->grabFrameBuffer().save(fileName);
		glWidget->resize(cW,cH);
		glWidget->updateGL();
	}
}

/**
 * Generate terrain according to the given image.
 *
 * sea :     blue (0, 0, 255)
 * river:    light blue (192, 192, 255)
 * plain:    green (0, 255, 0)
 * coast:    light green (192, 255, 192)
 * park:     pink (255, 128, 128)
 * water:    light blue (128, 128, 255)
 * mountain: brown (128, 0, 0)
 */
void MainWindow::onTerrainGeneration() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open sketch file..."), "", tr("Sketch Files (*.bmp *.jpg *.png)"));
	if (filename.isEmpty()) return;

	cv::Mat mat = cv::imread(filename.toUtf8().data());
	if (mat.rows != mat.cols) {
		std::cerr << "Only square shape of image is supported." << std::endl;
		return;
	}

	if (mat.rows != glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows) {
		std::cerr << "Terrain size is different." << std::endl;
	}

	if (mat.channels() != 3) {
		std::cerr << "Only the RGB color image is supported." << std::endl;
	}

	cv::Mat mat2;
	cv::flip(mat, mat2, 0);

	for (int y = 0; y < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows; ++y) {
		for (int x = 0; x < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols; ++x) {
			int b = mat2.at<cv::Vec3b>(y, x)[0];
			int g = mat2.at<cv::Vec3b>(y, x)[1];
			int r = mat2.at<cv::Vec3b>(y, x)[2];

			uchar value = 0;
			if (r == 0 && g == 0 && b == 255) {				// sea
				value = 0;
			} else if (r == 192 && g == 192 && b == 255) {	// river
				value = 1;
			} else if (r == 0 && g == 255 && b == 0) {		// plain
				value = 10;
			} else if (r == 192 && g == 255 && b == 192) {	// coast
				value = 9;
			} else if (r == 255 && g == 128 && b == 128) {	// park
				value = 8;
			} else if (r == 128 && g == 128 && b == 255) {	// water
				value = 7;
			} else if (r == 128 && g == 0 && b == 0) {		// mountain
				value = 11;
			} else {
				std::cerr << "Unknown color is found. red = " << r << ", green = " << g << ", blue = " << b << std::endl;
			}

			glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(y, x) = value;
		}
	}

	// create the transition area from the land to the sea
	std::vector<QVector2D> transitionAreas;
	for (int y = 0; y < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows; ++y) {
		for (int x = 0; x < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols; ++x) {
			uchar value = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(y, x);
			if (value > 1) continue;

			bool isTransitionArea = false;
			int size = 1;
			for (int yy = y - size; yy <= y + size && !isTransitionArea; ++yy) {
				if (yy < 0 || yy >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows) continue;
				for (int xx = x - size; xx <= x + size && !isTransitionArea; ++xx) {
					if (xx < 0 || xx >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
					uchar vv = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(yy, xx);
					if (vv > 1) {
						isTransitionArea = true;
						break;
					}
				}
			}

			if (isTransitionArea) {
				transitionAreas.push_back(QVector2D(x, y));
			}
		}
	}
	for (int i = 0; i < transitionAreas.size(); ++i) {
		glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(transitionAreas[i].y(), transitionAreas[i].x()) = 5;
	}

	glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.copyTo(origTerrain);


	urbanGeometry->adaptToTerrain();
	glWidget->updateGL();
}

void MainWindow::onUpdateMountain() {
	uchar original_mountain_elevation = 11;

	// once recover the original elevation
	origTerrain.copyTo(glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData);

	// randomly choose a point in the mountains area
	std::vector<QVector2D> mountains;
	for (int r = 0; r < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows; ++r) {
		for (int c = 0; c < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols; ++c) {
			uchar v = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(r, c);
			if (v != original_mountain_elevation) continue;

			int margin = 5;
			bool all_mountains = true;
			for (int rr = r - margin; rr <= r + margin && all_mountains; ++rr) {
				if (rr < 0 || rr >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows) continue;
				for (int cc = c - margin; cc <= c + margin && all_mountains; ++cc) {
					if (cc < 0 || cc >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
					uchar vv = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(rr, cc);
					if (vv != original_mountain_elevation) {
						all_mountains = false;
						break;
					}
				}
			}

			if (!all_mountains) continue;

			if (Util::genRand(0, 1) > 0.95f) {
				mountains.push_back(QVector2D(c, r));
			}
		}
	}

	float side = glWidget->vboRenderManager.side;
	float step = side / glWidget->vboRenderManager.vboTerrain.terrainLayer.imgResX;

	srand(clock());

	// create ups and downs on the mountains
	for (int i= 0; i < mountains.size(); ++i) {
		float x = mountains[i].x() * step / side;
		float y = mountains[i].y() * step / side;
		float change = Util::genRand(0, 0.01);
		float radi = Util::genRand(0.03, 0.07f);

		glWidget->vboRenderManager.vboTerrain.updateTerrain(x, y, change,radi);
	}

	urbanGeometry->adaptToTerrain();
	glWidget->updateGL();
}

/**
 * Segment the area according to the elevation which is stored in the terrainLayer matrix.
 *
 * 0 -- sea
 * 1 -- river
 * 7 -- water area
 * 8 -- park
 * 9 -- coast
 * 10 -- flat
 * 11~ -- mountains
 */
void MainWindow::onTerrainSegmentation() {
	std::vector<std::vector<Polygon2D> > polygonsSet;
	polygonsSet.resize(5);	// 0 -- flat / 1 -- coast / 2 -- park / 3 -- water / 4 -- mountains

	float side = glWidget->vboRenderManager.side;
	float step = side / glWidget->vboRenderManager.vboTerrain.terrainLayer.imgResX;

	std::cout << "start terrain segmentation..." << std::endl;

	for (int r = 0; r < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows - 1; ++r) {
		for (int c = 0; c < glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols - 1; ++c) {
			uchar value0 = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(r, c);

			if (value0 <= 1) {
				// try to find the closest area type
				bool done = false;
				for (int d = 1; d <= 10 && !done; ++d) {
					for (int rr = r - d; rr <= r + d && !done; ++rr) {
						int cc = c - d;
						if (rr < 0 || rr >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows || cc < 0 || cc >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
						uchar v = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(rr, cc);
						if (v > 1) {
							value0 = v;
							done = true;
							break;
						}

						cc = c + d;
						if (rr < 0 || rr >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows || cc < 0 || cc >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
						v = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(rr, cc);
						if (v > 1) {
							value0 = v;
							done = true;
						}
					}

					for (int cc = c - d; cc <= c + d && !done; ++cc) {
						int rr = r - d;
						if (rr < 0 || rr >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows || cc < 0 || cc >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
						uchar v = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(rr, cc);
						if (v > 1) {
							value0 = v;
							done = true;
							break;
						}

						rr = r + d;
						if (rr < 0 || rr >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.rows || cc < 0 || cc >= glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.cols) continue;
						v = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(rr, cc);
						if (v > 1) {
							value0 = v;
							done = true;
						}
					}
				}
			}

			uchar value1 = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(r + 1, c);
			uchar value2 = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(r + 1, c + 1);
			uchar value3 = glWidget->vboRenderManager.vboTerrain.terrainLayer.layerData.at<uchar>(r, c + 1);
			
			if (value0 <= 10 && value0 >= 7) {
				if (value1 < 8) value1 = value0;
				if (value2 < 8) value2 = value0;
				if (value3 < 8) value3 = value0;
			}
			if (value0 > 10) {
				if (value1 > 10) value1 = value0;
				if (value2 > 10) value2 = value0;
				if (value3 > 10) value3 = value0;
			}

			if (value1 == value0 && value2 == value0 && value3 == value0) {
				QVector2D pt0 = QVector2D(glWidget->vboRenderManager.vboTerrain.terrainLayer.minPos) + QVector2D((float)c * step, (float)r * step);
				QVector2D pt1 = pt0 + QVector2D(step, 0.0f);
				QVector2D pt2 = pt1 + QVector2D(0.0f, step);
				QVector2D pt3 = pt0 + QVector2D(0.0f, step);

				Polygon2D polygon;
				polygon.push_back(pt0);
				polygon.push_back(pt1);
				polygon.push_back(pt2);
				polygon.push_back(pt3);

				if (value0 == 10) {			// flat area
					polygonsSet[0].push_back(polygon);
				} else if (value0 == 9) {	// coast area
					polygonsSet[1].push_back(polygon);
				} else if (value0 == 8) {	// park
					polygonsSet[2].push_back(polygon);
				} else if (value0 == 7) {	// water area
					polygonsSet[3].push_back(polygon);
				} else if (value0 > 10) {	// mountains
					polygonsSet[4].push_back(polygon);
				}
			}
		}
	}

	std::cout << "The patches were categorized into five types." << std::endl;

	urbanGeometry->areas.clear();

	// merge the polygons
	for (int area_id = 0; area_id < 5; ++area_id) {
		while (!polygonsSet[area_id].empty()) {
			Polygon2D mergedPolygon = polygonsSet[area_id][0];
			polygonsSet[area_id].erase(polygonsSet[area_id].begin());

			bool loop = true;
			while (loop) {
				loop = false;
				for (int i = 0; i < polygonsSet[area_id].size(); ) {
					std::vector<Polygon2D> ret = mergedPolygon.union_(polygonsSet[area_id][i]);
					if (ret.size() == 1) {
						mergedPolygon = ret[0];
						polygonsSet[area_id].erase(polygonsSet[area_id].begin() + i);
						loop = true;
					} else {
						++i;
					}
				}
			}

			if (mergedPolygon.area() > 100000) {
				mergedPolygon.simplify(30.0f);
				RoadAreaPtr area = RoadAreaPtr(new RoadArea());
				area->area = mergedPolygon;
				area->hintLine.push_back(mergedPolygon.centroid());
				urbanGeometry->areas.add(area);

				std::cout << "An area (area id: " << area_id << ") was added." << std::endl;
			}
		}
	}

	std::cout << "Terrain segmentation has successfully completed" << std::endl;

	urbanGeometry->adaptToTerrain();
	glWidget->updateGL();
}

void MainWindow::onShowControlWidget() {
	controlWidget->show();
	addDockWidget(Qt::LeftDockWidgetArea, controlWidget);
}

void MainWindow::onShowPropertyWidget() {
	propertyWidget->show();
	addDockWidget(Qt::RightDockWidgetArea, propertyWidget);
}

void MainWindow::onDebug() {
	urbanGeometry->debug();
}
