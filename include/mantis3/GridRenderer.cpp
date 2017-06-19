/*
 * GridRenderer.cpp
 *
 *  Created on: Jun 18, 2017
 *      Author: pauvsi
 */

#include <mantis3/GridRenderer.h>

void CHECK_FRAMEBUFFER_STATUS()
{
	GLenum status;
	status = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
	switch(status) {
	case GL_FRAMEBUFFER_COMPLETE:
		break;

	case GL_FRAMEBUFFER_UNSUPPORTED:
		/* choose different formats */
		break;

	default:
		/* programming error; will fail on all hardware */
		fputs("Framebuffer Error\n", stderr);
		exit(-1);
	}
}

GridRenderer::GridRenderer(cv::Size sz) {

	setSize(sz);

	char *myargv [1];
	int myargc=1;
	myargv [0]=strdup ("Myappname");
	glutInit(&myargc, myargv);
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
	glutCreateWindow("FBO test");
	//glutDisplayFunc(display);
	//glutIdleFunc(glutPostRedisplay);
	glutHideWindow();

	glewInit();


	//set up
	glGenFramebuffers(1, &fb);
	glGenTextures(1, &color);
	glGenRenderbuffers(1, &depth);

	ROS_DEBUG("here1.25");

	glBindFramebuffer(GL_FRAMEBUFFER, fb);

	ROS_DEBUG("here1.5");
	glBindTexture(GL_TEXTURE_2D, color);
	glTexImage2D(   GL_TEXTURE_2D,
			0,
			GL_RGBA,
			sz.width, sz.height,
			0,
			GL_RGBA,
			GL_UNSIGNED_BYTE,
			NULL);

	ROS_DEBUG("here1");

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);

	glBindRenderbuffer(GL_RENDERBUFFER, depth);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, sz.width, sz.height);
	glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);

	CHECK_FRAMEBUFFER_STATUS();
}

GridRenderer::~GridRenderer() {

}

void GridRenderer::setColors(cv::Vec3i w, cv::Vec3i g, cv::Vec3i r)
{
	WHITE = cv::Vec3f(w[0] / 255.0, w[1] / 255.0, w[2] / 255.0);
	GREEN = cv::Vec3f(g[0] / 255.0, g[1] / 255.0, g[2] / 255.0);
	RED = cv::Vec3f(r[0] / 255.0, r[1] / 255.0, r[2] / 255.0);
}

cv::Mat GridRenderer::tfTransform2GLViewMat(tf::Transform trans)
{
	cv::Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64F);

	//column major
	viewMatrix.at<double>(0, 0) = trans.getBasis().getColumn(0).x();
	viewMatrix.at<double>(1, 0) = trans.getBasis().getColumn(0).y();
	viewMatrix.at<double>(2, 0) = trans.getBasis().getColumn(0).z();

	viewMatrix.at<double>(0, 1) = trans.getBasis().getColumn(1).x();
	viewMatrix.at<double>(1, 1) = trans.getBasis().getColumn(1).y();
	viewMatrix.at<double>(2, 1) = trans.getBasis().getColumn(1).z();

	viewMatrix.at<double>(0, 2) = trans.getBasis().getColumn(2).x();
	viewMatrix.at<double>(1, 2) = trans.getBasis().getColumn(2).y();
	viewMatrix.at<double>(2, 2) = trans.getBasis().getColumn(2).z();

	viewMatrix.at<double>(0, 3) = trans.getOrigin().x();
	viewMatrix.at<double>(1, 3) = trans.getOrigin().y();
	viewMatrix.at<double>(2, 3) = trans.getOrigin().z();

	viewMatrix.at<double>(3, 3) = 0;

	//gl transfer
	cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
	cvToGl.at<double>(0, 0) = 1.0f;
	cvToGl.at<double>(1, 1) = -1.0f; // Invert the y axis
	cvToGl.at<double>(2, 2) = -1.0f; // invert the z axis
	cvToGl.at<double>(3, 3) = 1.0f;
	viewMatrix = cvToGl * viewMatrix;

	cv::Mat glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
	cv::transpose(viewMatrix , glViewMatrix);

	return glViewMatrix;

}

void GridRenderer::setIntrinsic(cv::Mat_<float> K)
{
	this->K = K;
}

unsigned char* GridRenderer::getPixelData( int x1, int y1, int x2, int y2 )
{
	int y_low, y_hi;
	int x_low, x_hi;

	if ( y1 < y2 )
	{
		y_low = y1;
		y_hi  = y2;
	}
	else
	{
		y_low = y2;
		y_hi  = y1;
	}

	if ( x1 < x2 )
	{
		x_low = x1;
		x_hi  = x2;
	}
	else
	{
		x_low = x2;
		x_hi  = x1;
	}

	while ( glGetError() != GL_NO_ERROR )
	{
		std::string error;
		switch(glGetError()) {
		case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
		case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
		case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
		case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
		}

		ROS_DEBUG_STREAM("GL_" << error.c_str());
	}

	glReadBuffer( GL_BACK_LEFT );

	glDisable( GL_TEXTURE_2D );

	glPixelStorei( GL_PACK_ALIGNMENT, 1 );

	unsigned char *data = new unsigned char[ ( x_hi - x_low + 1 ) * ( y_hi - y_low + 1 ) * 3 ];

	glReadPixels( x_low, y_low, x_hi-x_low+1, y_hi-y_low+1, GL_BGR, GL_UNSIGNED_BYTE, data );


	if ( glGetError() != GL_NO_ERROR )
	{
		std::string error;
		switch(glGetError()) {
		case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
		case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
		case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
		case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
		case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
		}

		ROS_DEBUG_STREAM("GL_" << error.c_str());
		delete[] data;
		return 0;
	}
	else
	{
		return data;
	}
}

void setColor(cv::Vec3f col)
{
	glColor3f(col[2], col[1], col[0]);
}

void GridRenderer::generateGrid()
{
	//glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

	// draw a grid using quads
	//glLoadIdentity();                 // Reset the model-view matrix

	glBegin(GL_QUADS);                // Begin drawing the quads



	// draw y lines first (white)
	//double min = -(grid_size * grid_spacing / 2);
	//double max = (grid_size * grid_spacing / 2);

	//double ilt = inner_line_thickness/2.0;
	//double olt = outer_line_thickness/2.0;

	double grid_size = 9;
	double ilt=0.02;
	double olt=0.04;
	double grid_spacing = 0.3;
	double min = -(grid_size * grid_spacing / 2);
	double max = (grid_size * grid_spacing / 2);

	int line = 0;
	for(double x = min; x < max + grid_spacing; x += grid_spacing)
	{
		//ROS_DEBUG_STREAM("x: " << x);

		glColor3f(1, 1, 1);
		//setColor(WHITE);     // white

		//set the normal
		glNormal3f(0, 0, 1);

		if(line != 0 && line != grid_size)
		{
			glVertex3f( x - ilt, max, 0);
			glVertex3f( x + ilt, max, 0);
			glVertex3f( x + ilt, min, 0);
			glVertex3f( x - ilt, min, 0);
		}
		else
		{
			glVertex3f( x - olt, max, 0);
			glVertex3f( x + olt, max, 0);
			glVertex3f( x + olt, min, 0);
			glVertex3f( x - olt, min, 0);
		}

		line++;
	}

	line = 0;
	for(double y = min; y < max + grid_spacing; y += grid_spacing)
	{
		//ROS_DEBUG_STREAM("x: " << y);

		glColor3f(1, 1, 1);
		//setColor(WHITE);     // white

		//set the normal
		glNormal3f(0, 0, 1);

		if(line != 0 && line != grid_size)
		{
			glVertex3f(max, y - ilt, 0);
			glVertex3f(max, y + ilt, 0);
			glVertex3f(min, y + ilt, 0);
			glVertex3f(min, y - ilt, 0);
		}

		line++;
	}

	glEnd();  // End of drawing grid
	//glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)

}

cv::Mat GridRenderer::renderGrid()
{
	glBindTexture(GL_TEXTURE_2D, 0);
	glEnable(GL_TEXTURE_2D);
	glBindFramebuffer(GL_FRAMEBUFFER, fb);

	glViewport(0,0, size.width, size.height);

	glClearColor(0,0,0,0);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );


	//set up the intrinsic parameters
	glMatrixMode(GL_PROJECTION);
	/*glLoadIdentity();
	double fx = K(0);
	double fy = K(4);
	double cx = K(2);
	double cy = K(5);
	double W = size.width;
	double H = size.height;
	double zmin = 0.1;
	double zmax = 50;
	double s = 0;
	GLdouble perspMatrix[16]={2*fx/W,0,0,0,2*s/W,2*fy/H,0,0,2*(cx/W)-1,2*(cy/H)-1,(zmax+zmin)/(zmax-zmin),1,0,0,2*zmax*zmin/(zmin-zmax),0};
	glLoadMatrixd(perspMatrix);*/




	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();

	//move the camera
	cv::Mat glViewMatrix = tfTransform2GLViewMat(c2w);

	glLoadMatrixd(&glViewMatrix.at<double>(0, 0));

	// render the grid
	generateGrid();

	// get the image
	cv::Mat temp = cv::Mat(size.height, size.width, CV_8UC3);
	cv::Mat result;

	//glutSwapBuffers();
	glReadBuffer(GL_BACK);

	//glPixelStorei(GL_PACK_ALIGNMENT, (temp.step & 3) ? 1 : 4);
	glPixelStorei(GL_PACK_ROW_LENGTH, temp.step/temp.elemSize());
	// Read Image from buffer
	glReadPixels(0,0, temp.cols, temp.rows, GL_BGR, GL_UNSIGNED_BYTE,temp.data);

	// Process buffer so it matches correct format and orientation
	//cv::cvtColor(temp, tempImage, CV_BGR2RGB);

	cv::flip(temp, result, 0);
	//result = temp;

	return result;
}



