#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

int main(int num, char *args[]) {
	CvCapture *capture = NULL;
	IplImage *frame = NULL;
	char *MP4FileName;
	char *MP4SavePath;
	

	if(num==4){
		MP4FileName=args[1];
		MP4SavePath=args[2];
		const int space = atoi(args[3]);
		capture = cvCaptureFromAVI(MP4FileName);
		cvNamedWindow("asdf", 1);
		int count = 0;
		char tempFile[100] = { '\0' };
		while (true) {
			if (cvGrabFrame(capture)) {
				if (count%space == 0) {
					frame = cvRetrieveFrame(capture);
					//cvShowImage("asdf", frame);
					sprintf(tempFile, "%s//%d.jpg", MP4SavePath, count);
					cvSaveImage(tempFile, frame);
				}
				count++;
			}
			else {
				break;
			}
		}
		cvReleaseCapture(&capture);
	}
	return 0;		
}
