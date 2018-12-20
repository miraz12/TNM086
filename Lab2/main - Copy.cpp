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
osg::ref_ptr<osg::MatrixTransform> mSceneTrans;
osg::ref_ptr<osg::FrameStamp> mFrameStamp; //to sync osg animations across cluster
osg::ref_ptr<osg::Geometry> linesGeom;
osg::ref_ptr<osgUtil::LineSegmentIntersector> wandLine;
osg::ref_ptr<osg::Node> mCessnaModel;
osg::ref_ptr<osg::Node> mModel;
osg::ref_ptr<osg::Node> intersectedNode; //

//-----------------------
// function declarations
//-----------------------
void myInitOGLFun();
void myPreSyncFun();
void myPostSyncPreDrawFun();
void myDrawFun();
void myCleanUpFun();
void keyCallback(int key, int action);

void myEncodeFun();
void myDecodeFun();

void initOSG();
void createOSGScene();
void setupLightSource();
void calculateIntersections();

osg::Vec3d wand_start(0,-1,0);
osg::Vec3d wand_end(0,0,0);
glm::mat4 wand_matrix;
glm::mat4 head_matrix;
glm::vec3 wand_startPos;
glm::mat4 wand_startMat;

bool selecting;
bool intersecting;
bool moving;
//store each device's transform 4x4 matrix in a shared vector
sgct::SharedDouble curr_time(0.0);
sgct::SharedVector<glm::mat4> sharedTransforms;
sgct::SharedVector<bool> sharedButton;
sgct::SharedString sharedText;

int main( int argc, char* argv[] ){
  // Allocate
  gEngine = new sgct::Engine( argc, argv );

  // Bind your functions
  gEngine->setInitOGLFunction( myInitOGLFun );
  gEngine->setPreSyncFunction( myPreSyncFun );
  gEngine->setPostSyncPreDrawFunction( myPostSyncPreDrawFun );
  gEngine->setDrawFunction( myDrawFun );
  gEngine->setCleanUpFunction( myCleanUpFun );
	gEngine->setKeyboardCallbackFunction( keyCallback );

  // Init the engine
  if(!gEngine->init()){
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

void myInitOGLFun(){
  initOSG();
  createOSGScene();
  setupLightSource();

  glEnable(GL_DEPTH_TEST);
  //set intial values for selecting
  selecting = false;


  //only store the tracking data on the master node
  if( !gEngine->isMaster() ) return;

  for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++){
    sgct::SGCTTracker * trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);

    for(size_t j=0; j<trackerPtr->getNumberOfDevices(); j++){
      sgct::SGCTTrackingDevice * devicePtr = trackerPtr->getDevicePtr(j);

      if( devicePtr->hasSensor() ){
        sharedTransforms.addVal( glm::mat4(1.0f) );
      }
      if( devicePtr->hasButtons() ){
        for( int idx = 0 ; idx < devicePtr->getNumberOfButtons() ; ++idx ){
          sharedButton.addVal(0);
        }
      }
    }
  }
}

void myPreSyncFun(){

  // Only master does things in pre-sync; slaves return
  if( !gEngine->isMaster() ) return;

  curr_time.setVal( sgct::Engine::getTime() );

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
          sharedButton.setValAt(idx, devicePtr->getButton(idx));

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

  sharedText.setVal(message.str());
}

void myPostSyncPreDrawFun(){
  //update the frame stamp in the viewer to sync all
  //time based events in osg
  mFrameStamp->setFrameNumber( gEngine->getCurrentFrameNumber() );
  mFrameStamp->setReferenceTime( curr_time.getVal() );
  mFrameStamp->setSimulationTime( curr_time.getVal() );
  mViewer->setFrameStamp( mFrameStamp.get() );
  mViewer->advance( curr_time.getVal() ); //update

  bool point = false;
  bool crosshair = false;

  //Update position if button is pressed
  if(sharedButton.getSize()) {
    if(sharedButton.getValAt(0)) {
       //point mode
       point = true;
	   moving = true;
    }
    else if(sharedButton.getValAt(1)) {
       //crosshair mode
       crosshair = true;
	   moving = true;
    }
    else if(sharedButton.getValAt(2)) {
      //Selection of model
      selecting = true;

    }
    else {
	  selecting = false;
	  moving = false;
    }
  }
  // Draw wand in OSG
  if( sharedTransforms.getSize() > WAND_SENSOR_IDX ){
    wand_matrix = sharedTransforms.getValAt(WAND_SENSOR_IDX);

    glm::vec3 wand_position = glm::vec3(wand_matrix*glm::vec4(0,0,0,1));
    //glm::quat wand_orientation = glm::quat_cast(wand_matrix);
    glm::mat3 wand_orientation = glm::mat3(wand_matrix);

    glm::vec3 start = wand_position;
    glm::vec3 end = wand_position + wand_orientation * glm::vec3(0,0,-10);
    wand_start = osg::Vec3(start.x, start.y, start.z);
    wand_end = osg::Vec3(end.x, end.y, end.z);

    osg::Vec3Array* vertices = new osg::Vec3Array();
    vertices->push_back(wand_start);
    vertices->push_back(wand_end);
    linesGeom->setVertexArray(vertices);
    wandLine->setStart(wand_start);
    wandLine->setEnd(wand_end);
  }
  else {
    //Debug drawing for wand even if there is no VRPN server
    osg::Vec3Array* vertices = new osg::Vec3Array();
    vertices->push_back(wand_start);
    vertices->push_back(wand_end);
    linesGeom->setVertexArray(vertices);
    wandLine->setStart(wand_start);
    wandLine->setEnd(wand_end);
  }
  //movement - only if we have a head to move ;)
  if( sharedTransforms.getSize() > HEAD_SENSOR_IDX) {
	if (!moving) {
	  //store initial position of wand for deadzone calculation
	  wand_startPos = glm::vec3(wand_matrix*glm::vec4(0, 0, 0, 1)); 
	}

    wand_matrix = sharedTransforms.getValAt(WAND_SENSOR_IDX);
    
	glm::vec3 wand_position = glm::vec3(wand_matrix*glm::vec4(0,0,0,1));
    glm::mat3 wand_orientation = glm::mat3(wand_matrix);

    head_matrix = sharedTransforms.getValAt(HEAD_SENSOR_IDX);
    glm::vec3 head_position = glm::vec3(head_matrix*glm::vec4(0,0,0,1));
	
	//intial static speed factor
	float speedFactor = 0.1*gEngine->getDt();
	
	//user sets speed, deadzone is 10 cm from original position
	speedFactor = (	length(wand_startPos - wand_position) < 0.1 ? 0 : length(wand_startPos - wand_position)/50);
	
	//if we pull the control towards us it should go backwards
	int direction = (	length(wand_startPos - head_position) > length(wand_position - head_position) ) ? -1 : 1;
	speedFactor *= direction;
    //Move the world in the opposite direction for the movement effect 
	if(point) {
		glm::vec3 translation = (glm::mat3(wand_matrix) * glm::vec3(0, 0, -1)*speedFactor);
      mSceneTrans->postMult(osg::Matrix::translate(
		  -osg::Vec3(translation.x, translation.y, translation.z)));
    }
    else if (crosshair) {
		glm::vec3 translation = normalize(head_position - wand_position)*speedFactor;
      mSceneTrans->postMult(osg::Matrix::translate(
		  osg::Vec3(translation.x, translation.y, translation.z)));
    }
  }
	if (!selecting) {
		// save wand matrix for manipulation
		wand_startMat = wand_matrix;
	}
  //traverse if there are any tasks to do
  if (!mViewer->done()){
    mViewer->eventTraversal();
    mViewer->updateTraversal();
    calculateIntersections();
  }
}

void calculateIntersections() {
  //visit all nodes and look for intersection
  osgUtil::IntersectionVisitor visitor;
  visitor.setIntersector(wandLine);
  mRootNode->accept(visitor);

  if(!intersectedNode && wandLine->containsIntersections()) {
	//get intersection, store it and change the color of the object to a highlight yellow color
	//we store it
	osgUtil::LineSegmentIntersector::Intersection intersectionInfo = wandLine->getFirstIntersection();
    osg::NodePath nodePath = intersectionInfo.nodePath;

    intersecting = true;
    for (osg::NodePath::iterator it = nodePath.begin() ; it != nodePath.end(); ++it) {
      if((*it) == mCessnaModel || (*it) == mModel) {
        intersectedNode = (*it);
      }
    }

    osg::ref_ptr<osg::Material> mat = (osg::Material*)intersectedNode->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);

    if(!mat) {
      mat = new osg::Material();
    }
    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 0, 1.0));
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(1, 1, 0, 1.0));

    intersectedNode->getOrCreateStateSet()->setAttributeAndModes(mat.get(), osg::StateAttribute::OVERRIDE);
  }
  else if(selecting && intersectedNode) {
	//selection button pressed - make the object green and make it follow the wand
    osg::ref_ptr<osg::Material> mat = (osg::Material*)intersectedNode->getOrCreateStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
    mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0, 1, 0, 1.0));
    mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0, 1, 0, 1.0));
    intersectedNode->getOrCreateStateSet()->setAttributeAndModes(mat.get(), osg::StateAttribute::OVERRIDE);

	//use the difference between the starting wand orientation and current position to determine the transformation
    glm::mat4 diff = wand_startMat;
    glm::mat4 diffInv = inverse(wand_matrix);

	  osg::ref_ptr < osg::MatrixTransform > parent = intersectedNode->getParent(0)->asTransform()->asMatrixTransform();
   
	    parent->postMult(osg::Matrix(glm::value_ptr(inverse(diff*diffInv))));
    

    wand_startMat = wand_matrix;
  }
  else if(!wandLine->containsIntersections()) {
    mModel->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::MATERIAL);
    mCessnaModel->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::MATERIAL);
    intersectedNode = NULL;
  }
  wandLine->reset();
}

void myDrawFun() {
  const int * curr_vp = gEngine->getCurrentViewportPixelCoords();
  mViewer->getCamera()->setViewport(curr_vp[0], curr_vp[1], curr_vp[2], curr_vp[3]);
  mViewer->getCamera()->setProjectionMatrix( osg::Matrix( glm::value_ptr(gEngine->getCurrentViewProjectionMatrix() ) ));

  mViewer->renderingTraversals();

	// draw text with OpenGL
	float textVerticalPos = static_cast<float>(gEngine->getCurrentWindowPtr()->getYResolution()) - 100.0f;
	int fontSize = 12;

	glColor3f(1.0f, 1.0f, 1.0f);
	sgct_text::print(sgct_text::FontManager::instance()->getFont( "SGCTFont", fontSize ),
		120.0f, textVerticalPos,
		sharedText.getVal().c_str() );
}

void myEncodeFun(){
  sgct::SharedData::instance()->writeDouble( &curr_time );
  sgct::SharedData::instance()->writeVector( &sharedTransforms );
	sgct::SharedData::instance()->writeString( &sharedText );
}

void myDecodeFun(){
  sgct::SharedData::instance()->readDouble( &curr_time );
  sgct::SharedData::instance()->readVector( &sharedTransforms );
	sgct::SharedData::instance()->readString( &sharedText );
}

void myCleanUpFun(){
  sgct::MessageHandler::instance()->print("Cleaning up osg data...\n");
  delete mViewer;
  mViewer = NULL;
}

void keyCallback(int key, int action) {

  if( !gEngine->isMaster() ) return;

  switch (key) {
  case 'Q':
  case 'q':
  case SGCT_KEY_ESC:
    gEngine->terminate();
    break;
  case SGCT_KEY_W:
    wand_start.y() += 1*gEngine->getDt();
    wand_end.y() += 1*gEngine->getDt();
    break;
  case SGCT_KEY_S:
    wand_start.y() -= 1*gEngine->getDt();
    wand_end.y() -= 1*gEngine->getDt();
    break;
  case SGCT_KEY_A:
    wand_start.x() -= 1*gEngine->getDt();
    wand_end.x() -= 1*gEngine->getDt();
    break;
  case SGCT_KEY_D:
    wand_start.x() += 1*gEngine->getDt();
    wand_end.x() += 1*gEngine->getDt();
    break;
  case SGCT_KEY_Z:
    wand_start.z() += 1*gEngine->getDt();
    wand_end.z() += 1*gEngine->getDt();
    break;
  case SGCT_KEY_X:
    wand_start.z() -= 1*gEngine->getDt();
    wand_end.z() -= 1*gEngine->getDt();
    break;
	//buttons for debugging selecting
  case SGCT_KEY_Y:
    selecting = true;
    break;
  case SGCT_KEY_U:
    selecting = false;
    break;


}

void initOSG(){
  mRootNode = new osg::Group();
  osg::Referenced::setThreadSafeReferenceCounting(true);

  // Create the osgViewer instance
  mViewer = new osgViewer::Viewer;

  // Create a time stamp instance
  mFrameStamp	= new osg::FrameStamp();

  //run single threaded when embedded
  mViewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

  // Set up osgViewer::GraphicsWindowEmbedded for this context
  osg::ref_ptr< ::osg::GraphicsContext::Traits > traits =
    new osg::GraphicsContext::Traits;

  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphicsWindow =
    new osgViewer::GraphicsWindowEmbedded(traits.get());

  mViewer->getCamera()->setGraphicsContext(graphicsWindow.get());

  wandLine = new osgUtil::LineSegmentIntersector(wand_start, wand_end);

  //SGCT will handle the near and far planes
  mViewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
  mViewer->getCamera()->setClearColor( osg::Vec4( 0.0f, 0.0f, 0.0f, 0.0f) );

  //disable osg from clearing the buffers that will be done by SGCT
  GLbitfield tmpMask = mViewer->getCamera()->getClearMask();
  mViewer->getCamera()->setClearMask(tmpMask & (~(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)));

  wandLine->setStart(wand_start);
  wandLine->setEnd(wand_end);

  mViewer->setSceneData(mRootNode.get());
}

osg::Geode* createWand(){

  osg::Geode* geode = new osg::Geode();

  linesGeom = new osg::Geometry();

  osg::Vec3Array* vertices = new osg::Vec3Array();
  vertices->push_back(osg::Vec3(0, 0, 0));
  vertices->push_back(osg::Vec3(1, 0, 0));
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

void createOSGScene(){

  mRootNode->addChild(createWand());

  osg::ref_ptr<osg::MatrixTransform> mModelTrans;
  osg::ref_ptr<osg::MatrixTransform> mCessnaTrans;


  mSceneTrans		= new osg::MatrixTransform();
  mModelTrans		= new osg::MatrixTransform();
  mCessnaTrans	= new osg::MatrixTransform();

  // rotate osg model to match sgct coordinate system
  mModelTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
                                           1.0f, 0.0f, 0.0f));

  mCessnaTrans->preMult(osg::Matrix::rotate(glm::radians(-90.0f),
                                           1.0f, 0.0f, 0.0f));
  mRootNode->addChild( mSceneTrans.get() );
  mSceneTrans->addChild( mModelTrans.get() );
  mSceneTrans->addChild( mCessnaTrans.get() );

  sgct::MessageHandler::instance()->print("Loading model airplane.ive'...\n");
  sgct::MessageHandler::instance()->print("and cessna.org'...\n");
  mModel = osgDB::readNodeFile("airplane.ive");
  mCessnaModel = osgDB::readNodeFile("cessna.osg");

  if ( mModel.valid() && mCessnaModel.valid()){
    sgct::MessageHandler::instance()->print("Models loaded successfully!\n");

    mModelTrans->addChild(mModel.get());
    mCessnaTrans->addChild(mCessnaModel.get());

    //get the bounding box
    osg::ComputeBoundsVisitor cbv;
    osg::BoundingBox &bb(cbv.getBoundingBox());
    mModel->accept( cbv );

    osg::Vec3f tmpVec;
    tmpVec = bb.center();
    tmpVec.x() += 20;

    // translate model center to origin
    mModelTrans->postMult(osg::Matrix::translate( -tmpVec ) );

    // scale model to a manageable size
    double scale = 0.2 / bb.radius();
    mModelTrans->postMult(osg::Matrix::scale( scale, scale, scale ));

    sgct::MessageHandler::instance()->print("airplane bounding sphere center:\tx=%f\ty=%f\tz=%f\n", tmpVec[0], tmpVec[1], tmpVec[2] );
    sgct::MessageHandler::instance()->print("airplane bounding sphere radius:\t%f\n", bb.radius() );

    //same as above for the cessna
    mCessnaModel->accept( cbv );

    tmpVec = bb.center();
    tmpVec.x() -= 20;
    // translate model center to origin
    mCessnaTrans->postMult(osg::Matrix::translate( -tmpVec ) );

    // scale model to a manageable size
    scale = 0.1 / bb.radius();
    mCessnaTrans->postMult(osg::Matrix::scale( scale, scale, scale ));


	sgct::MessageHandler::instance()->print("cessna bounding sphere center:\tx=%f\ty=%f\tz=%f\n", tmpVec[0], tmpVec[1], tmpVec[2]);
	sgct::MessageHandler::instance()->print("cessna bounding sphere radius:\t%f\n", bb.radius());
    //disable face culling
    mCessnaModel->getOrCreateStateSet()->setMode( GL_CULL_FACE,
                                            osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
    mModel->getOrCreateStateSet()->setMode( GL_CULL_FACE,
                                            osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
  }
  else
    sgct::MessageHandler::instance()->print("Failed to read model!\n");

}

void setupLightSource(){
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
