#include <osg/Version>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osgUtil/Simplifier>
#include <osgUtil/LineSegmentIntersector>
#include <osg/PositionAttitudeTransform>

#include <cmath>


class IntersectCallback : public osg::NodeCallback
{

public:
  IntersectCallback(osg::PositionAttitudeTransform* cessnaTransform, osg::Light* light, osg::Geometry* line)
  {
    m_cessnaTransform = cessnaTransform;
    m_light = light;
    m_line = line;
    state = false;

    
  };

  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  {

    //printf("plbbo\n");
    m_intersector = new osgUtil::LineSegmentIntersector(osg::Vec3(-100., 0., 0.), osg::Vec3(100., 0., 0.));
    
    osgUtil::IntersectionVisitor iv( m_intersector);


    m_cessnaTransform->accept( iv );

    if (m_intersector->containsIntersections())
    {

        m_light->setAmbient(osg::Vec4(0.5, 0.8, 0.2, 0.0));
        m_light->setDiffuse(osg::Vec4(0.1, 0.9, 0.1, 0.0));
        m_light->setSpecular(osg::Vec4(0.1, 0.2, 0.3, 0.0));
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.0f,0.2f,0.9f,1.0f));
        m_line->setColorArray(colors, osg::Array::BIND_OVERALL);
        state = true;
      }
      else
      {

        m_light->setAmbient(osg::Vec4(0.0, 0.1, 0.1, 0.0));
        m_light->setDiffuse(osg::Vec4(0.1, 0.1, 0.2, 0.0));
        m_light->setSpecular(osg::Vec4(0.0, 0.0, 1.0, 0.0));
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.9f,0.2f,0.3f,1.0f));
        m_line->setColorArray(colors, osg::Array::BIND_OVERALL);
        state = false;
      }

      printf("plbbo\n");

    traverse(node, nv);
    //node->accept(nv);
    // osgUtil::IntersectionVisitor iv = static_cast<oosgUtil::IntersectionVisitor>(nv);

    // if (intersector->containsIntersections())
    // {
    // }

  };

private:
  osg::PositionAttitudeTransform* m_cessnaTransform;
  osg::Light* m_light;
  osg::Geometry* m_line;
  osgUtil::LineSegmentIntersector* m_intersector;
  bool state;
};


// int IntersectionCallback (osg::PositionAttitudeTransform* cessnaTransform)
// {
//   osgUtil::LineSegmentIntersector* intersector = new osgUtil::LineSegmentIntersector(osg::Vec3(-100., 0., 0.), osg::Vec3(100., 0., 0.));
//   osgUtil::IntersectionVisitor iv( intersector);
//   cessnaTransform->accept( iv );

//   if (intersector->containsIntersections())
//   {
//     printf("plbbo\n");
//   }

//   return
// }



osg::AnimationPath* createAnimationPath(float radius, float time)
{
  osg::AnimationPath* pathCessna = new osg::AnimationPath;
  pathCessna->setLoopMode(osg::AnimationPath::LOOP);

  unsigned int numSamples = 32;
  float deltaYaw = 2.0f * osg::PI / ((float) numSamples - 1.0f);

  float deltaTime = time / (float)numSamples;
  for (int i = 0; i < numSamples; ++i)
  {
     float yaw = deltaYaw * (float)i;
     osg::Vec3 pos(sinf(yaw) *radius, cosf(yaw)*radius, 0.0f);
     osg::Quat rot( -yaw, osg::Z_AXIS );
     pathCessna->insert( deltaTime * (float)i, osg::AnimationPath::ControlPoint(pos, rot));
  } 
   return pathCessna;
}

int main(int argc, char *argv[]){

      printf("plbbo\n");

  
  osg::ref_ptr<osg::Group> root = new osg::Group;

#if 1
  /// Line ---

  osg::Vec3 line_p0 (-100, 0, 0);
  osg::Vec3 line_p1 ( 100, 0, 0);
  
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
  vertices->push_back(line_p0);
  vertices->push_back(line_p1);
  
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(0.9f,0.2f,0.3f,1.0f));

  osg::ref_ptr<osg::Geometry> linesGeom = new osg::Geometry();
  linesGeom->setVertexArray(vertices);
  linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
  
  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));
  
  osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
  lineGeode->addDrawable(linesGeom);
  lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  
  root->addChild(lineGeode);
  
  /// ---
#endif

  
  // Add your stuff to the root node here...
  osg::HeightField* heightField = new osg::HeightField();
  heightField->allocate(100, 100);
  //heightField->setOrigin(osg::Vec3(-heightMap->s() / 2, -heightMap->t() / 2, 0));
  heightField->setXInterval(1.0f);
  heightField->setYInterval(1.0f);
  for(int y = 0; y < 100; ++y)
  {
    for(int x = 0; x < 100; ++x)
    {
      heightField->setHeight(x, y, sin((x + y)/5.0)/2.0);
    }
  }

  osg::Geode* heightFieldGeode = new osg::Geode();
  heightFieldGeode->addDrawable(new osg::ShapeDrawable(heightField));


  osg::Texture2D* tex = new osg::Texture2D(osgDB::readImageFile("frog.jpg"));
  tex->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
  tex->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
  tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
  tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

  heightFieldGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);

  root->addChild(heightFieldGeode);



  osg::Node* spaceshipModel = osgDB::readNodeFile("spaceship.osg");

  osg::Node* spaceshipModelMed = dynamic_cast<osg::Node*>(spaceshipModel->clone(osg::CopyOp::DEEP_COPY_ALL));
  osg::Node* spaceshipModelLow = dynamic_cast<osg::Node*>(spaceshipModel->clone(osg::CopyOp::DEEP_COPY_ALL));

  osgUtil::Simplifier simplifier;
  simplifier.setSampleRatio(0.5);
  spaceshipModelMed->accept(simplifier);

  simplifier.setSampleRatio(0.1);
  spaceshipModelLow->accept(simplifier);


  osg::LOD* lodNode1 = new osg::LOD;
  lodNode1->addChild(spaceshipModel, 0.0, 20.0);
  lodNode1->addChild(spaceshipModelMed, 20.0, 100.0);
  lodNode1->addChild(spaceshipModelLow, 100.0, 1000.0);

  osg::PositionAttitudeTransform* spaceshipTransform = new osg::PositionAttitudeTransform;
  spaceshipTransform->setPosition(osg::Vec3(0.0, 0.0, 30.0));
  spaceshipTransform->addChild(lodNode1);



  root->addChild(spaceshipTransform);

  osg::Node* cessnaModel = osgDB::readNodeFile("cessna.osg");
  osg::PositionAttitudeTransform* cessnaTransform = new osg::PositionAttitudeTransform;
  cessnaTransform->setPosition(osg::Vec3(0.0, -50.0, 10.0));
  cessnaTransform->addChild(cessnaModel);

  root->addChild(cessnaTransform);

  osg::AnimationPathCallback* apcb = new osg::AnimationPathCallback;
  apcb->setAnimationPath(createAnimationPath(50.0f, 6.0f));

  cessnaTransform->setUpdateCallback(apcb);




  osg::Light *light1 = new osg::Light();
  light1->setPosition(osg::Vec4(0.0, 0.0, 10.0, 1.0));
  light1->setAmbient(osg::Vec4(0.5, 0.0, 0.5, 0.0));
  light1->setDiffuse(osg::Vec4(0.1, 0.2, 0.3, 0.0));
  light1->setSpecular(osg::Vec4(0.1, 0.2, 0.3, 0.0));
  //light1->setDirection(osg::Vec3d(0.0, 0.0, 1.0));
  osg::LightSource* lightSource1 = new osg::LightSource;
  lightSource1->setLight(light1);
  lightSource1->setStateSetModes(*root->getOrCreateStateSet(), osg::StateAttribute::ON);
  root->addChild(lightSource1);


  osg::Light *light2 = new osg::Light();
  light2->setPosition(osg::Vec4(-50.0, 20.0, 50.0, 1.0));
  light2->setAmbient(osg::Vec4(0.0, 0.5, 0.5, 0.0));
  light2->setDiffuse(osg::Vec4(0.5, 0.1, 0.8, 0.0));
  light2->setSpecular(osg::Vec4(0.1, 0.2, 0.3, 0.0));
  osg::LightSource* lightSource2 = new osg::LightSource;
  lightSource2->setLight(light2);
  lightSource2->setStateSetModes(*root->getOrCreateStateSet(), osg::StateAttribute::ON);
  root->addChild(lightSource2);


  
  root->setUpdateCallback(new IntersectCallback(cessnaTransform, light1, linesGeom));

  // Optimizes the scene-graph
  //osgUtil::Optimizer optimizer;
  //optimizer.optimize(root);
  

  // Set up the viewer and add the scene-graph root
  osgViewer::Viewer viewer;
  viewer.setSceneData(root);

  osg::ref_ptr<osg::Camera> camera = new osg::Camera;
  camera->setProjectionMatrixAsPerspective(60.0, 1.0, 0.1, 100.0);
  camera->setViewMatrixAsLookAt (osg::Vec3d(0.0, 0.0, 2.0),
                                 osg::Vec3d(0.0, 0.0, 0.0),
                                 osg::Vec3d(0.0, 1.0, 0.0));
  camera->getOrCreateStateSet()->setGlobalDefaults();
  viewer.setCamera(camera);
  
  return viewer.run();
}
