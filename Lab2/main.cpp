
#include <sgct.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>

#include <osg/ComputeBoundsVisitor>

#include <osg/Version>
#include <osg/LineSegment>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osgUtil/Simplifier>
#include <osgUtil/LineSegmentIntersector>
#include <osg/PositionAttitudeTransform>

#include <cmath>

#include <sgct.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>

#include <osg/ComputeBoundsVisitor>
#include <osg/Material>
#include <glm/gtx/matrix_interpolation.hpp>

sgct::Engine * gEngine;

#define WAND_SENSOR_IDX 0
#define HEAD_SENSOR_IDX 1

// OSG stuff
osgViewer::Viewer * mViewer;
osg::ref_ptr<osg::Group> mRootNode;
osg::ref_ptr<osg::MatrixTransform> mSGCTTrans;
osg::ref_ptr<osg::MatrixTransform> mSceneTrans;
osg::ref_ptr<osg::FrameStamp> mFrameStamp; //to sync osg animations across cluster
osg::ref_ptr<osg::Geometry> linesGeom;
osg::ref_ptr<osgUtil::LineSegmentIntersector> lineSegment;
osg::ref_ptr<osg::Node> cessna;

// callbacks
void myInitOGLFun();
void myPreSyncFun();
void myPostSyncPreDrawFun();
void myDrawFun();
void myEncodeFun();
void myDecodeFun();
void myCleanUpFun();
void keyCallback(int key, int action);

// other functions
void initOSG();
void createOSGScene();
void setupLightSource();
osg::Geode* createWand();

//variables to share across cluster
sgct::SharedDouble curr_time(0.0);
sgct::SharedDouble dist(-2.0);
sgct::SharedVector<glm::mat4> sharedTransforms;
sgct::SharedString sharedText;

sgct::SharedBool wireframe(false);
sgct::SharedBool info(false);
sgct::SharedBool stats(false);
sgct::SharedBool takeScreenshot(false);
sgct::SharedBool light(true);

//Set as global to be able to move wand linge in non vr enviorment.
osg::Vec3 wand_start = osg::Vec3(0, 0, 1);
osg::Vec3 wand_end = osg::Vec3(0, 0, 0);

// Simple initial navigation based on arrow buttons
bool arrowButtons[4];
enum directions { FORWARD = 0, BACKWARD, LEFT, RIGHT };
const double navigation_speed = 1.0;

class IntersectCallback : public osg::NodeCallback
{

public:
    IntersectCallback(osg::MatrixTransform* transformC, osg::MatrixTransform* transformM)
    {
        m_transformC = transformC;
        m_transformM = transformM;
    };

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {

        osgUtil::IntersectionVisitor iv(lineSegment);
        mRootNode->accept(iv);

        if (lineSegment->containsIntersections())
        {
            osgUtil::LineSegmentIntersector::Intersection intersectionInfo = lineSegment->getFirstIntersection();
            
            osg::ref_ptr<osg::Material> mat = new osg::Material();
            mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1.0));
            mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1.0));

            osg::ref_ptr<osg::Node> inter;
            for (osg::NodePath::iterator it = intersectionInfo.nodePath.begin(); it != intersectionInfo.nodePath.end(); ++it) 
            {
                if ((*it) == m_transformM || (*it) == m_transformC)
                {
                    inter = (*it);
                }
            }

            if (inter)
            {
                inter->getOrCreateStateSet()->setAttributeAndModes(mat.get(), osg::StateAttribute::OVERRIDE);
                printf("bloop\n");
            }
        }
        else
        {
            printf("NO bloop\n");
            osg::ref_ptr<osg::Material> mat = new osg::Material();
            mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1.0));
            mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 1, 1.0));
            m_transformC->getOrCreateStateSet()->setAttributeAndModes(mat.get(), osg::StateAttribute::OVERRIDE);
            m_transformM->getOrCreateStateSet()->setAttributeAndModes(mat.get(), osg::StateAttribute::OVERRIDE);
        }

        lineSegment->reset();
    }

private:
    osg::MatrixTransform* m_transformC;
    osg::MatrixTransform* m_transformM;

};

int main( int argc, char* argv[] ) {
  gEngine = new sgct::Engine( argc, argv );

  gEngine->setInitOGLFunction( myInitOGLFun );
  gEngine->setPreSyncFunction( myPreSyncFun );
  gEngine->setPostSyncPreDrawFunction( myPostSyncPreDrawFun );
  gEngine->setDrawFunction( myDrawFun );
  gEngine->setCleanUpFunction( myCleanUpFun );
  gEngine->setKeyboardCallbackFunction( keyCallback );

  //fix incompability with warping and OSG
  sgct_core::ClusterManager::instance()->setMeshImplementation( sgct_core::ClusterManager::DISPLAY_LIST );

  for(int i=0; i<4; i++)
    arrowButtons[i] = false;

  if( !gEngine->init() ) {
    delete gEngine;
    return EXIT_FAILURE;
  }

  sgct::SharedData::instance()->setEncodeFunction( myEncodeFun );
  sgct::SharedData::instance()->setDecodeFunction( myDecodeFun );

  // Main loop
  gEngine->render();

  // Clean up
  delete gEngine;

  // Exit program
  exit( EXIT_SUCCESS );
}

void myInitOGLFun() {
  initOSG();
  createOSGScene();
  setupLightSource();

  //only store the tracking data on the master node
  if( !gEngine->isMaster() ) return;

  for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++){
    sgct::SGCTTracker * trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);

    for(size_t j=0; j<trackerPtr->getNumberOfDevices(); j++){
      sgct::SGCTTrackingDevice * devicePtr = trackerPtr->getDevicePtr(j);

      if (devicePtr->hasSensor())
        sharedTransforms.addVal( glm::mat4(1.0f) );
    }
  }
  sharedText.setVal(" "); //< Space since SGCT hangs otherwise
}

void createOSGScene() {

  mRootNode->addChild(createWand());

  osg::ref_ptr<osg::Node>            mModel;
  osg::ref_ptr<osg::Node>            mCessna;
  osg::ref_ptr<osg::MatrixTransform> mModelTrans;
  osg::ref_ptr<osg::MatrixTransform> mCessnaTrans;


  mSGCTTrans         = new osg::MatrixTransform();
  mSceneTrans        = new osg::MatrixTransform();
  mModelTrans        = new osg::MatrixTransform();
  mCessnaTrans =        new osg::MatrixTransform();


  //rotate osg coordinate system to match sgct
  mModelTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
                                           1.0f, 0.0f, 0.0f));
  mCessnaTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
      1.0f, 0.0f, 0.0f));





  mRootNode->addChild( mSGCTTrans.get() );
  mSGCTTrans->addChild( mSceneTrans.get() );
  mSceneTrans->addChild( mModelTrans.get() );
  mSceneTrans->addChild(mCessnaTrans.get() );

  sgct::MessageHandler::instance()->print("Loading model 'airplane.ive'...\n");
  mModel = osgDB::readNodeFile("airplane.ive");
  mCessna = osgDB::readNodeFile("cessna.osg");

  if (!mModel.valid() && !mCessna.valid()) {
    sgct::MessageHandler::instance()->print("Failed to read model!\n");
    return;
  }

  sgct::MessageHandler::instance()->print("Model loaded successfully!\n");
  mModelTrans->addChild(mModel.get());
  mCessnaTrans->addChild(mCessna.get());

  //get the bounding box
  osg::ComputeBoundsVisitor cbv;
  osg::BoundingBox &bb(cbv.getBoundingBox());
  mModel->accept( cbv );
  mCessna->accept(cbv);
  cessna = mCessna;

  osg::Vec3f tmpVec;
  tmpVec = bb.center();
  tmpVec.x() += 20;

  // translate model center to origin
  mModelTrans->postMult(osg::Matrix::translate( -tmpVec ) );
  tmpVec = bb.center();
  tmpVec.x() -= 20;
  mCessnaTrans->postMult(osg::Matrix::translate(-tmpVec));


  // scale model to a manageable size
  double scale = 0.1 / bb.radius();
  mModelTrans->postMult(osg::Matrix::scale(scale, scale, scale));
  mCessnaTrans->postMult(osg::Matrix::scale(scale, scale, scale));

  sgct::MessageHandler::instance()->print("Model bounding sphere center:\tx=%f\ty=%f\tz=%f\n", tmpVec[0], tmpVec[1], tmpVec[2] );
  sgct::MessageHandler::instance()->print("Model bounding sphere radius:\t%f\n", bb.radius() );
  
  mRootNode->setUpdateCallback(new IntersectCallback(mCessnaTrans, mModelTrans));

  //disable face culling
  mModel->getOrCreateStateSet()->setMode( GL_CULL_FACE,
                                          osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
  mCessna->getOrCreateStateSet()->setMode(GL_CULL_FACE,
      osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
}

void myPreSyncFun() {
  if (!gEngine->isMaster())
    return;

  curr_time.setVal( sgct::Engine::getTime() );

  if( arrowButtons[FORWARD] )
    dist.setVal( dist.getVal() + (navigation_speed * gEngine->getDt()));

  if( arrowButtons[BACKWARD] )
    dist.setVal( dist.getVal() - (navigation_speed * gEngine->getDt()));

  std::stringstream message;

  size_t index = 0;
  for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++){
    sgct::SGCTTracker * trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);

    for(size_t j = 0; j < trackerPtr->getNumberOfDevices(); j++){
      sgct::SGCTTrackingDevice * devicePtr = trackerPtr->getDevicePtr(j);

      message << "Device " << i << " on tracker " << j << std::endl;

      if( devicePtr->hasSensor() ){
        sharedTransforms.setValAt( index, devicePtr->getWorldTransform() );
        message << "Position:" << std::endl << "  "
                << devicePtr->getPosition().x << ", "
                << devicePtr->getPosition().y << ", "
                << devicePtr->getPosition().z << std::endl;
        message << "Euler angles:" << std::endl << "  "
                << devicePtr->getEulerAngles().x << ", "
                << devicePtr->getEulerAngles().y << ", "
                << devicePtr->getEulerAngles().z << std::endl;
        index++;
      }

      if( devicePtr->hasButtons() ){
        message << "Buttons:" << std::endl << "  ";
        for( int idx = 0 ; idx < devicePtr->getNumberOfButtons() ; ++idx ){
          message << devicePtr->getButton(idx) ? "1" : "0";
        }
        message << std::endl;
      }

      if( devicePtr->hasAnalogs() ){
        message << "Analogs:" << std::endl << "  ";
        for( int idx = 0 ; idx < devicePtr->getNumberOfAxes() ; ++idx ){
          message << "  " << devicePtr->getAnalog(idx) << std::endl;
        }
      }

      message << std::endl;
    }
  }

  if (message.str().size() < 2)
    message << " "; //< Space since SGCT hangs otherwise
  sharedText.setVal(message.str());
}

void myPostSyncPreDrawFun() {
  gEngine->setWireframe(wireframe.getVal());
  gEngine->setDisplayInfoVisibility(info.getVal());
  gEngine->setStatsGraphVisibility(stats.getVal());

  if (takeScreenshot.getVal()) {
    gEngine->takeScreenshot();
    takeScreenshot.setVal(false);
  }

  if (light.getVal())
    mRootNode->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                               osg::StateAttribute::ON |
                                               osg::StateAttribute::OVERRIDE);
  else
    mRootNode->getOrCreateStateSet()->setMode( GL_LIGHTING,
                                               osg::StateAttribute::OFF |
                                               osg::StateAttribute::OVERRIDE);

  // Simple initial navigation based on arrow buttons
  mSceneTrans->setMatrix(osg::Matrix::translate(0.0, 0.0, dist.getVal()));

  // SGCT internal transformation from configuration file
  mSGCTTrans->setMatrix(osg::Matrix(glm::value_ptr(gEngine->getModelMatrix())));

  //update the frame stamp in the viewer to sync all
  //time based events in osg
  mFrameStamp->setFrameNumber( gEngine->getCurrentFrameNumber() );
  mFrameStamp->setReferenceTime( curr_time.getVal() );
  mFrameStamp->setSimulationTime( curr_time.getVal() );
  mViewer->setFrameStamp( mFrameStamp.get() );
  mViewer->advance( curr_time.getVal() ); //update


  // Update wand in OSG
  if( sharedTransforms.getSize() > WAND_SENSOR_IDX ){
    glm::mat4 wand_matrix = sharedTransforms.getValAt(WAND_SENSOR_IDX);

    glm::vec3 wand_position = glm::vec3(wand_matrix*glm::vec4(0,0,0,1));
    //glm::quat wand_orientation = glm::quat_cast(wand_matrix);
    glm::mat3 wand_orientation = glm::mat3(wand_matrix);

    glm::vec3 start = wand_position;
    glm::vec3 end = wand_position + wand_orientation * glm::vec3(0,0,-1);

    osg::Vec3d wand_start = osg::Vec3(start.x, start.y, start.z);
    osg::Vec3d wand_end = osg::Vec3(end.x, end.y, end.z);

    osg::Vec3Array* vertices = new osg::Vec3Array();
    vertices->push_back(wand_start);
    vertices->push_back(wand_end);
    linesGeom->setVertexArray(vertices);
    lineSegment->setStart(wand_start);
    lineSegment->setEnd(wand_end);


  }
  else //For non vr env
  {
      osg::Vec3Array* vertices = new osg::Vec3Array();
      vertices->push_back(wand_start);
      vertices->push_back(wand_end);
      linesGeom->setVertexArray(vertices);
      lineSegment->setStart(wand_start);
      lineSegment->setEnd(wand_end);
  }

  if (sharedTransforms.getSize() > HEAD_SENSOR_IDX) 
  {


  }



  //traverse if there are any tasks to do
  if (!mViewer->done()) {
    mViewer->eventTraversal();
    //update travelsal needed for pagelod object like terrain data etc.
    mViewer->updateTraversal();
  }
}

void myDrawFun() {
  glLineWidth(2.0f);

  const int * curr_vp = gEngine->getCurrentViewportPixelCoords();
  mViewer->getCamera()->setViewport(curr_vp[0], curr_vp[1], curr_vp[2], curr_vp[3]);
  mViewer->getCamera()->setProjectionMatrix( osg::Matrix( glm::value_ptr(gEngine->getCurrentViewProjectionMatrix() ) ));

  mViewer->renderingTraversals();

	// draw text with OpenGL
	float textVerticalPos = static_cast<float>(gEngine->getCurrentWindowPtr()->getYResolution()) - 100.0f;
	int fontSize = 12;

	glColor3f(1.0f, 1.0f, 1.0f);
	/*sgct_text::print(sgct_text::FontManager::instance()->getFont( "SGCTFont", fontSize ),
		sgct_text::TextAlignMode::TOP_LEFT,
		120.0f, textVerticalPos,
		sharedText.getVal().c_str() );*/
}

void myEncodeFun() {
  sgct::SharedData::instance()->writeDouble( &curr_time );
  sgct::SharedData::instance()->writeVector( &sharedTransforms );
	sgct::SharedData::instance()->writeString( &sharedText );
  sgct::SharedData::instance()->writeBool( &wireframe );
  sgct::SharedData::instance()->writeBool( &info );
  sgct::SharedData::instance()->writeBool( &stats );
  sgct::SharedData::instance()->writeBool( &takeScreenshot );
  sgct::SharedData::instance()->writeBool( &light );
}

void myDecodeFun() {
  sgct::SharedData::instance()->readDouble( &curr_time );
  sgct::SharedData::instance()->readVector( &sharedTransforms );
  sgct::SharedData::instance()->readString( &sharedText );
  sgct::SharedData::instance()->readBool( &wireframe );
  sgct::SharedData::instance()->readBool( &info );
  sgct::SharedData::instance()->readBool( &stats );
  sgct::SharedData::instance()->readBool( &takeScreenshot );
  sgct::SharedData::instance()->readBool( &light );
}

void myCleanUpFun() {
  sgct::MessageHandler::instance()->print("Cleaning up osg data...\n");
  delete mViewer;
  mViewer = NULL;
}

void keyCallback(int key, int action) {
  if(!gEngine->isMaster())
    return;
  const double dt = gEngine->getDt();

  switch (key) {
  /*case 'W':
    if (action == SGCT_PRESS)
      wireframe.toggle();
    break;
      
  case 'S':
    if(action == SGCT_PRESS)
      stats.toggle();
    break;*/
  //Wand line debug movement----------------
  case 'W':
      //wand_start.y() += 1 * dt;
      wand_end.y() += 1 * dt;
      break;
  case 'A':
      //wand_start.x() -= 1 * dt;
      wand_end.x() -= 1 * dt;
      break;
  case 'S':
      //wand_start.y() -= 1 * dt;
      wand_end.y() -= 1 * dt;
      break;
  case 'D':
      //wand_start.x() += 1 * dt;
      wand_end.x() += 1 * dt;
      break;
  //------------------------------------------
  case 'I':
    if(action == SGCT_PRESS)
      info.toggle();
    break;

  case 'L':
    if(action == SGCT_PRESS)
      light.toggle();
    break;

 

  case 'Q':
    if(action == SGCT_PRESS)
      gEngine->terminate();
    break;

  case 'P':
  case SGCT_KEY_F10:
    if(action == SGCT_PRESS)
      takeScreenshot.setVal( true );
    break;

  case SGCT_KEY_UP:
    arrowButtons[FORWARD] = ((action == SGCT_REPEAT || action == SGCT_PRESS) ? true : false);
    break;

  case SGCT_KEY_DOWN:
    arrowButtons[BACKWARD] = ((action == SGCT_REPEAT || action == SGCT_PRESS) ? true : false);
    break;
  }
}

void initOSG() {
  mRootNode = new osg::Group();
  osg::Referenced::setThreadSafeReferenceCounting(true);

  // Create the osgViewer instance
  mViewer = new osgViewer::Viewer;

  // Create a time stamp instance
  mFrameStamp    = new osg::FrameStamp();

  //run single threaded when embedded
  mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

  // Set up osgViewer::GraphicsWindowEmbedded for this context
  osg::GraphicsContext::Traits *traits =
    new osg::GraphicsContext::Traits;

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphicsWindow =
    new osgViewer::GraphicsWindowEmbedded(traits);

  mViewer->getCamera()->setGraphicsContext(graphicsWindow.get());

  //SGCT will handle the near and far planes
  mViewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
  mViewer->getCamera()->setClearColor( osg::Vec4( 0.0f, 0.0f, 0.0f, 0.0f) );

  //disable osg from clearing the buffers that will be done by SGCT
  GLbitfield tmpMask = mViewer->getCamera()->getClearMask();
  mViewer->getCamera()->setClearMask(tmpMask & (~(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)));

  mViewer->setSceneData(mRootNode.get());
}

void setupLightSource() {
  osg::Light * light0 = new osg::Light;
  osg::Light * light1 = new osg::Light;
  osg::LightSource* lightSource0 = new osg::LightSource;
  osg::LightSource* lightSource1 = new osg::LightSource;

  light0->setLightNum( 0 );
  light0->setPosition( osg::Vec4( 5.0f, 5.0f, 10.0f, 1.0f ) );
  light0->setAmbient( osg::Vec4( 0.3f, 0.3f, 0.3f, 1.0f ) );
  light0->setDiffuse( osg::Vec4( 0.8f, 0.8f, 0.8f, 1.0f ) );
  light0->setSpecular( osg::Vec4( 0.1f, 0.1f, 0.1f, 1.0f ) );
  light0->setConstantAttenuation( 1.0f );

  lightSource0->setLight( light0 );
  lightSource0->setLocalStateSetModes( osg::StateAttribute::ON );
  lightSource0->setStateSetModes( *(mRootNode->getOrCreateStateSet()), osg::StateAttribute::ON );

  light1->setLightNum( 1 );
  light1->setPosition( osg::Vec4( -5.0f, -2.0f, 10.0f, 1.0f ) );
  light1->setAmbient( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
  light1->setDiffuse( osg::Vec4( 0.5f, 0.5f, 0.5f, 1.0f ) );
  light1->setSpecular( osg::Vec4( 0.2f, 0.2f, 0.2f, 1.0f ) );
  light1->setConstantAttenuation( 1.0f );

  lightSource1->setLight( light1 );
  lightSource1->setLocalStateSetModes( osg::StateAttribute::ON );
  lightSource1->setStateSetModes( *(mRootNode->getOrCreateStateSet()), osg::StateAttribute::ON );

  mRootNode->addChild( lightSource0 );
  mRootNode->addChild( lightSource1 );
}

osg::Geode* createWand() {

  osg::Geode* geode = new osg::Geode();


  
  linesGeom = new osg::Geometry();
  lineSegment = new osgUtil::LineSegmentIntersector(wand_start, wand_end);

  osg::Vec3Array* vertices = new osg::Vec3Array();
  vertices->push_back(wand_start);
  vertices->push_back(wand_end);
  linesGeom->setVertexArray(vertices);

  osg::Vec4Array* colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(0.3f,0.7f,0.4f,1.0f));
  linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

  osg::Vec3Array* normals = new osg::Vec3Array;
  normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
  linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);

  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));
  geode->addDrawable(linesGeom);

  return geode;
}
