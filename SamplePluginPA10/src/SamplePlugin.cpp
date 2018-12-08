#include "SamplePlugin.hpp"
//#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <functional>

#include <rw/math/Q.hpp>
#include <rw/math.hpp>
#include <math.h>

#include <iostream>
#include <fstream>

using namespace rw::common;
using namespace rw::math;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;
using namespace cv;
using namespace std::placeholders;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_checkBox,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_Vision  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}
SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}
void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);
	// Auto load workcell
//	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/alexdupond/ROVI/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/miols15/ROVI/ROVI/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);


	// Load Lena image
	Mat im, image;
//	im = imread("/home/alexdupond/ROVI/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    im = imread("/home/miols15/ROVI/ROVI/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
    cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}
void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
	// Add the texture render to this workcell if there is a frame for texture
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
	}
	// Add the background render to this workcell if there is a frame for texture
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
	}

	// Create a GLFrameGrabber if there is a camera frame with a Camera property set
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	if (cameraFrame != NULL) {
		if (cameraFrame->getPropertyMap().has("Camera")) {
			// Read the dimensions and field of view
			double fovy;
			int width,height;
			std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
			std::istringstream iss (camParam, std::istringstream::in);
			iss >> fovy >> width >> height;
			// Create a frame grabber
			_framegrabber = new GLFrameGrabber(width,height,fovy);
			SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
			_framegrabber->init(gldrawer);
		}
	}
    }
}
void SamplePlugin::loadMotions(){
 // string path = "/home/alexdupond/ROVI/SamplePluginPA10/motions/MarkerMotionMedium.txt";
  string path = "/home/miols15/ROVI/ROVI/SamplePluginPA10/motions/MarkerMotionMedium.txt";
  std::ifstream motionFile(path);


  if(motionFile.is_open()){
    double X, Y, Z, R0, P0, Y0;
    while(motionFile >> X >> Y >> Z >> R0 >> P0 >> Y0){
        //log().info() << "X = " << X << ", Y = " << Y << ", Z = " << Z << ", R0 = " << R0 << ", P0 = " << P0 << ", Y0 = " << Y0 << "\n";
        const Vector3D<double> d(X, Y, Z);
        const RPY<double> rpy(R0, P0, Y0);
        const Transform3D<double> trans(d, rpy.toRotation3D());
        _motions.push_back(trans);
    }
    motionFile.close();
  }
}
void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}
Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}
void SamplePlugin::btnPressed() {
    QObject *obj = sender();

    if(obj==_btn0){
		log().info() << "Button 0\n";
        // Loads the motion
        loadMotions();

		// Set a new texture (one pixel = 1 mm)
        string device_name = "PA10";
        rw::models::Device::Ptr device = _wc->findDevice("PA10");
        if(device == nullptr) {
            RW_THROW("Device " << device_name << " was not found!");
         }
        rw::math::Q startQ(7, 0, -0.65, 0, 1.8, 0, 0.42, 0);
        device->setQ(startQ, _state);
        getRobWorkStudio()->setState(_state);
        _motionIndex = 0;

		Image::Ptr image;
//		image = ImageLoader::Factory::load("/home/alexdupond/ROVI/SamplePluginPA10/markers/Marker1.ppm");
        image = ImageLoader::Factory::load("/home/miols15/ROVI/ROVI/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
//		image = ImageLoader::Factory::load("/home/alexdupond/ROVI/SamplePluginPA10/backgrounds/color1.ppm");
        image = ImageLoader::Factory::load("/home/miols15/ROVI/ROVI/SamplePluginPA10/backgrounds/color1.ppm");

        _bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off

		if (!_timer->isActive())
            _timer->start(100); // run 100 = 10 Hz // deltat
		else
			_timer->stop();
	} else if(obj==_spinBox){
        _deltaT = 10 * _spinBox->value();
        log().info() << "_deltaT; " << _deltaT << "\n";
	}
    else if (obj==_checkBox){
        if ( _checkBox->isChecked() ){
            _singlePtracking = true;
        }
        else{
            _singlePtracking = false;
        }
    }
}
// This function calculates delta U as in Equation 4.13. The output class is a velocity screw as that is a 6D vector with a positional and rotational part
// What a velocity screw really is is not important to this class. For our purposes it is only a container.
rw::math::VelocityScrew6D<double> calculateDeltaU(const rw::math::Transform3D<double>& baseTtool, const rw::math::Transform3D<double>& baseTtool_desired) {
    // Calculate the positional difference, dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
}
//GetZimg
rw::math::Jacobian getZimg (rw::math::Jacobian imageJacobian, rw::kinematics::Frame* tcp_frame,
                            rw::kinematics::State state, rw::models::Device::Ptr device){
    // Get robot Jacobian
    rw::math::Jacobian robotJ = device->baseJframe(tcp_frame, state);
    // Get  RBC =R-Cam_Base Transposed
    rw::math::Transform3D<> baseTtool_rw = device->baseTframe(tcp_frame, state);
    Rotation3D<double> Rot = baseTtool_rw.R().inverse();
    int sizeJ = 6;
    rw::math::Jacobian Sp(sizeJ,sizeJ);
    // Make S(q) = (RBC, 0) (0,RBC)
    for (int row = 0; row <= sizeJ-1; row++){
        for ( int col = 0; col <= sizeJ-1; col++){
            if (row <= 2 && col <= 2 )
                Sp(row,col) = Rot(row,col);
            else if(row > 2 && col > 2 )
                Sp(row,col) = Rot(row-3, col-3);
            else
                Sp(row,col) = 0;
        }
    }
    return (imageJacobian*Sp)*robotJ;  // Zimg= Image Jacobian * S(p) * Robot-Jacobian(q)
}
double D(double xORy,double z, double f){
  return (f*xORy)/z;
}
// Get IImage J
rw::math::Jacobian imageJ (double x,double y,double z,double f){
    double u = (f * x)/z;
    double v = (f * y)/z;
    vector<double> ju {-f/z,     0,  u/z,                (u*v)/f, -( (pow(f,2)+pow(u,2))/f ),   v };
    vector<double> jv {   0,  -f/z,  v/z,  (pow(f,2)+pow(u,2))/f,                   -(u*v)/f,  -u };
    vector<vector<double>> jujv {ju,jv};
    rw::math::Jacobian imageJ(2,6);
    for (int i = 0; i<imageJ.size1();i++)
    {
        for(int j = 0; j<imageJ.size2();j++)
        {
            imageJ(i,j)=jujv[i][j];
        }
    }
    return imageJ;
}
// Get IImage J
rw::math::Jacobian imageJ2 (double x,double y,double z, double x1,double y1, double z1,double f){
    double u = (f * x)/z;
    double v = (f * y)/z;
    vector<double> ju {-f/z,     0,  u/z,                (u*v)/f, -( (pow(f,2)+pow(u,2))/f ),   v };
    vector<double> jv {   0,  -f/z,  v/z,  (pow(f,2)+pow(u,2))/f,                   -(u*v)/f,  -u };

    double u1 = (f * x1)/z1;
    double v1 = (f * y1)/z1;
    vector<double> ju1 {-f/z1,     0,  u1/z1,                (u1*v1)/f, -( (pow(f,2)+pow(u1,2))/f ),   v1 };
    vector<double> jv1 {   0,  -f/z1,  v/z1,  (pow(f,2)+pow(u1,2))/f,                   -(u1*v1)/f,  -u1 };

    vector<vector<double>> jujv {ju,jv,ju1,jv1};

    rw::math::Jacobian imageJ(4,6);
    for (int i = 0; i<imageJ.size1();i++)
    {
        for(int j = 0; j<imageJ.size2();j++)
        {
            imageJ(i,j)=jujv[i][j];
        }
    }
    return imageJ;
  }
// Get IImage J
rw::math::Jacobian imageJ3 (double x,double y,double z, double x1,double y1, double z1,double x2,double y2, double z2,double f){
      double u = (f * x)/z;
      double v = (f * y)/z;
      vector<double> ju {-f/z,     0,  u/z,                (u*v)/f, -( (pow(f,2)+pow(u,2))/f ),   v };
      vector<double> jv {   0,  -f/z,  v/z,  (pow(f,2)+pow(u,2))/f,                   -(u*v)/f,  -u };

      double u1 = (f * x1)/z1;
      double v1 = (f * y1)/z1;
      vector<double> ju1 {-f/z1,     0,  u1/z1,                (u1*v1)/f, -( (pow(f,2)+pow(u1,2))/f ),   v1 };
      vector<double> jv1 {   0,  -f/z1,  v1/z1,  (pow(f,2)+pow(u1,2))/f,                   -(u1*v1)/f,  -u1 };

      double u2 = (f * x2)/z2;
      double v2 = (f * y2)/z2;
      vector<double> ju2 {-f/z2,     0,  u2/z2,                (u2*v2)/f, -( (pow(f,2)+pow(u2,2))/f ),   v2 };
      vector<double> jv2 {   0,  -f/z2,  v2/z2,  (pow(f,2)+pow(u2,2))/f,                   -(u2*v2)/f,  -u2 };

      vector<vector<double>> jujv {ju,jv,ju1,jv1,ju2,jv2};

      rw::math::Jacobian imageJ(6,6);
      for (int i = 0; i<imageJ.size1();i++)
      {
          for(int j = 0; j<imageJ.size2();j++)
          {
              imageJ(i,j)=jujv[i][j];
          }
      }
      return imageJ;
    }
void SamplePlugin::timer() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        State state = _state;
        Frame* markerFrame = _wc->findFrame("Marker");
        MovableFrame* mFrame = (MovableFrame*)markerFrame;

        if(_motionIndex == (int)_motions.size())
          _motionIndex = 0;

        Transform3D<double> trans = _motions[_motionIndex];
        mFrame->setTransform(trans, state);

        // Convert to OpenCV image
        Mat im = toOpenCVImage(image);
        Mat imflip;
        cv::flip(im, imflip, 0);

        // Show in QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

        Point centerPoint = findCenterMaker1(im);
        log().info() << " \n Centerpoint:" << centerPoint.x - 1024/2 -8 << "," << centerPoint.y - 768/2 +76  <<"\n";

        // Robo devices
        string device_name = "PA10";
        rw::models::Device::Ptr device = _wc->findDevice("PA10");
        if(device == nullptr) {
          RW_THROW("Device " << device_name << " was not found!");
        }

        rw::kinematics::Frame* tcp_frame = _wc->findFrame("Camera");
        if(tcp_frame == nullptr) {
          RW_THROW("TCP 'Camera' frame not found!");
        }

        Transform3D<double> MarkerToCam = tcp_frame->fTf(markerFrame,state);
        rw::math::Q deltaQ;
        if (_checkBox->isChecked() ){
             double x, y, z, f;
             if ( _Vision->isChecked() ) //Vision enable
             {
                 //Camera is 1024 * 768
                 centerPoint = findCenterMaker1(im);
                 Vector3D<double> p1 (centerPoint.x - 1024/2, //x
                                      centerPoint.y - 768/2,  // y
                                      0.5);                   // z

                 Vector3D<double> point1 = MarkerToCam * p1; // Transform to Camera frame
                 x = point1(0);
                 y = point1(1);
                 z = point1(2);
                 f = 832;

                 rw::math::Jacobian imJ = imageJ(x,y, z,f);
                 vector<Vector2D<double>> uv = { {D(x,z,f), D(y,z,f)} };

                 cout << "uv " << uv[0] << endl;
                 if (_motionIndex == 0){//First time make desired = 0,0
                     Vector3D<double> p1first(512,384,0);
                     Vector3D<double> point1first = MarkerToCam * p1first;
                     x = point1first(0); // 100 = golden factor
                     y = point1first(1);
                     z = point1first(2);
                     f = 832;
                    vector<Vector2D<double>> uv1 = { {D(x,z,f), D(y,z,f)} };
                     _uv1pDesired = uv1;
                 }
                 Eigen::Matrix<double,2, 1> duv;
                 for(int i= 0; i < uv.size(); i++)
                 {
                     auto d =  _uv1pDesired[i]-uv[i];
                     cout << "uvDesired - uv "<< _uv1pDesired[i] << " uv: " << uv[i] << " = " << d << endl;
                     duv(i*2, 0)   = d(0);
                     duv(i*2+1, 0) = d(1);
                 }
                 log().info() << "xyz\n "<< point1 << "uv:\n" << uv[0] << "uvDesired: \n"<< _uv1pDesired[0] << "\n";

                 //log().info() << "duv: \n" << duv <<"\n point2FollowCamframe: \n" << MarkerToCam.P() << "\n";
                 rw::math::Jacobian Zimg = getZimg(imJ, tcp_frame, state, device);
                 auto Z = Zimg.e() * Zimg.e().transpose(); // Zimg = 2x7 Z =2*2
                 rw::math::Q y (LinearAlgebra::pseudoInverse(Z) * duv); //6*1
                 rw::math::Q initDeltaQ ( Zimg.e().transpose() * y.e() );
                 deltaQ = initDeltaQ; //7*6 * 6*1
             }
             else{ // single point no Vision
                 x = MarkerToCam.P()(0);
                 y = MarkerToCam.P()(1);
                 z = MarkerToCam.P()(2);
                 f=1;

                 rw::math::Jacobian imJ = imageJ(x, y, z,f);
                 vector<Vector2D<double>> uv = { {D(x,z,f), D(y,z,f)} };
                 if (_motionIndex == 0)
                     _uvDesired = uv;


                 Eigen::Matrix<double,2, 1> duv;
                 for(int i= 0; i < uv.size(); i++)
                 {
                     auto d =  _uvDesired[i]-uv[i];
                     duv(i*2, 0) = d(0);
                     duv(i*2+1, 0) = d(1);
                 }
                 log().info() << " duv: \n" << duv <<"\n point2FollowCamframe: \n" << MarkerToCam.P() << "\n";
                 rw::math::Jacobian Zimg = getZimg(imJ, tcp_frame, state, device);
                 auto Z = Zimg.e() * Zimg.e().transpose();
                 rw::math::Q y (LinearAlgebra::pseudoInverse(Z) * duv);
                 rw::math::Q initDeltaQ ( Zimg.e().transpose() * y.e() );
                 deltaQ = initDeltaQ;
             }
        }
        else { // 3 point tracking:
            /// Perfect point follow:
            Rotation3D<double> R(1,0,0,0,1,0,0,0,1);
            Vector3D<double> p1(0.15,0.15,0);
            Vector3D<double> p2(-0.15,0.15,0);
            Vector3D<double> p3(0.15,-0.15,0);

            Transform3D<double> T1(p1,R);
            Transform3D<double> T2(p2,R);
            Transform3D<double> T3(p3,R);

            Vector3D<double> point1 = MarkerToCam * p1;
            Transform3D<double> point2 = MarkerToCam * T2;
            Transform3D<double> point3 = MarkerToCam * T3;

            double x1 = point1(0) , y1 = point1(1) , z1 = point1(2), f=1;
            double x2 = point2.P()(0) , y2 = point2.P()(1) , z2 = point2.P()(2);
            double x3 = point3.P()(0) , y3 = point3.P()(1) , z3 = point3.P()(2);

            rw::math::Jacobian imJ = imageJ3(x1, y1, z1, x2, y2, z2, x3, y3, z3, f);
            vector<Vector2D<double>> uv = { {D(x1,z1,f), D(y1,z1,f)}, {D(x2,z2,f), D(y2,z2,f)},{ D(x3,z3,f), D(y3,z3,f)} };

            if (_motionIndex == 0)
            {
                _uvDesired = uv;
            }

            Eigen::Matrix<double,6, 1> duv;
            for(int i= 0; i < uv.size(); i++)
            {
                auto d =  _uvDesired[i]-uv[i];
                duv(i*2, 0) = d(0);
                duv(i*2+1, 0) = d(1);
            }
            log().info() << " duv: \n" << duv <<"\n point2FollowCamframe: \n" << point1 << "\n";
            rw::math::Jacobian Zimg = getZimg(imJ, tcp_frame, state, device);
            auto Z = Zimg.e() * Zimg.e().transpose(); // Zimg = 6x7 Z =6*6
            rw::math::Q y (LinearAlgebra::pseudoInverse(Z) * duv); //6*1
            rw::math::Q initDeltaQ ( Zimg.e().transpose() * y.e() );
            deltaQ = initDeltaQ; //7*6 * 6*1
        }

        double deltaT = _deltaT/1000;
        rw::math::Q v(7, deltaQ(0)/deltaT, deltaQ(1)/deltaT, deltaQ(2)/deltaT, deltaQ(3)/deltaT, deltaQ(4)/deltaT, deltaQ(5)/deltaT, deltaQ(6)/deltaT);
        rw::math::Q vLimit = device->getVelocityLimits();
        log().info() << "\n Velocity: \n" << v << "\n";
        log().info() << "deltaQ:\n" << deltaQ << "\n";
        // Speed Limits
        for(int i = 0; i < 7;i++ ) //
        {
          if( v(i) > vLimit(i))
          {
            log().info() << "Velocity Limited hitted at joint: " << i << " speed: "<< v(i) << "\n";
            deltaQ(i) = vLimit(i)*deltaT;
            log().info() << "dQ Capped to: \n" << deltaQ(i) << "\n";
          }else if(v(i) < -vLimit(i)){
            log().info() << "Velocity Limited hitted at joint: " << i << " speed: "<< v(i) << "\n";
            deltaQ(i) = -vLimit(i)*deltaT;
            log().info() << "dQ Capped to: \n" << deltaQ(i) << "\n";
          }
        }


        rw::math::Q q = device->getQ(state);
        rw::math::Q newQ = q+deltaQ;
        device->setQ(newQ, state);
        getRobWorkStudio()->setState(state);
        _motionIndex++;
	}
}
void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}
Point SamplePlugin::findCenterMaker1(Mat &image){
  Mat hsv, blured_img;
  Mat detected_egdes, dst, src, gray;

  cvtColor(image, hsv, CV_BGR2HSV);


  vector<Mat> hsv_planes;
  split(hsv, hsv_planes);
  Mat h = hsv_planes[0];
  Mat s = hsv_planes[1];
  Mat v = hsv_planes[2];

  int lowThreshold = 10;
  int ratio = 3;
  int kernal_size = 3;

  // bluring the image
  medianBlur(h, blured_img, 21);

  // Using canny to find edges
  Canny(blured_img, detected_egdes, lowThreshold, lowThreshold*ratio, kernal_size);

  // Setting up vectors and findeing the contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours( detected_egdes, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0, 0) );

  // Creating points vector finding the center
  vector<Point> points;

  // Red point if the center pixel of the red circle
  Point redpoint(0,0);
  for( int i = 0; i < (int)contours.size(); i++ )
      {
        double area = contourArea(contours[i]);
        double perimeter = arcLength(contours[i], false);
        double circle_detect = (4*CV_PI*area)/(pow(perimeter, 2));
        Moments m = moments(contours[i], true);
        int cx = m.m10/m.m00;
        int cy = m.m01/m.m00;
        Point center(cx,cy);

        if(circle_detect > 0.8 && (area > 2500)){
          points.push_back(center);
          Scalar color = image.at<Vec3b>(cy, cx);
          if(color[2] > 80)
            redpoint = center;

      }
  }

  // Finding the center of circles (coc)
  Point coc(0,0);
  if(points.size()){
    for (int i = 0; i < points.size(); i++) {
      coc.x += points[i].x;
      coc.y +=points[i].y;
    }
    coc.x = coc.x/points.size();
    coc.y = coc.y/points.size();

    return coc;
  }

  return coc;
}
