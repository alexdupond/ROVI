#include "SamplePlugin.hpp"
//#include <rw/rw.hpp>


#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <functional>

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
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

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
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/alexdupond/ROVI/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);


	// Load Lena image
	Mat im, image;
	im = imread("/home/alexdupond/ROVI/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
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
  string path = "/home/alexdupond/ROVI/SamplePluginPA10/motions/MarkerMotionMedium.txt";
  std::ifstream motionFile(path);


  if(motionFile.is_open()){
    double X, Y, Z, R0, P0, Y0;
    while(motionFile >> X >> Y >> Z >> R0 >> P0 >> Y0){
        log().info() << "X = " << X << ", Y = " << Y << ", Z = " << Z << ", R0 = " << R0 << ", P0 = " << P0 << ", Y0 = " << Y0 << "\n";
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

		Image::Ptr image;
		image = ImageLoader::Factory::load("/home/alexdupond/ROVI/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load("/home/alexdupond/ROVI/SamplePluginPA10/backgrounds/color1.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(100); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

// Get IImage J
rw::math::Jacobian imageJ (double x,double y,double z,double f){
    double u = (f * x)/z;
    double v = (f * y)/z;
    vector<double> du {-f/z,     0,  u/z,                (u*v)/f, -( (pow(f,2)+pow(u,2))/f ),   v };
    vector<double> dv {   0,  -f/z,  v/z,  (pow(f,2)+pow(u,2))/f,                   -(u*v)/f,  -u };
    vector<vector<double>> dUdV {du,dv};
    rw::math::Jacobian imageJ(2,6);
    for (int i = 0; i<imageJ.size1();i++)
    {
        for(int j = 0; j<imageJ.size2();j++)
        {
            imageJ(i,j)=dUdV[i][j];
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

        State state = _wc->getDefaultState();
        Frame* markerFrame = _wc->findFrame("Marker");
        MovableFrame* mFrame = (MovableFrame*)markerFrame;


        if(_motionIndex == (int)_motions.size())
          _motionIndex = 0;

        Transform3D<double> trans = _motions[_motionIndex];
        mFrame->setTransform(trans, state);
        stateChangedListener(state);
        getRobWorkStudio()->setState(_state);


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
        _motionIndex++;
        findCenterMaker1(im);

        /// from Image Jacobian to new q.
        ///
        ///
        rw::models::Device::Ptr device = _wc->findDevice("PA10");
        if(device == nullptr) {
          RW_THROW("Device " << device_name << " was not found!");
        }

        rw::kinematics::Frame* tcp_frame = wc->findFrame(device_name + ".Joint7");
        if(tcp_frame == nullptr) {
          RW_THROW("TCP frame not found!");
        }

        // Get image Jacobian
        double z=0.5, f=823;
        rw::math::Jacobian imJ = imageJ(x,y,z,f);

        // Get Zimg
        // q=currentstate, tcp_frame
        rw::math::Jacobian Zimg = getZimg(imJ, q, tcp_frame, state, device);
        cout <<"Zimg:\n"<< Zimg << endl;

        ///

	}
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}


void SamplePlugin::findCenterMaker1(Mat &image){
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
  if(points.size()){
    Point coc(0,0);
    for (int i = 0; i < points.size(); i++) {
      coc.x += points[i].x;
      coc.y +=points[i].y;
    }
    coc.x = coc.x/points.size();
    coc.y = coc.y/points.size();

    log().info() << "Center of circles = " << coc.x << ", " << coc.y << "\n";
  }

}
