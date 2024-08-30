/*
 * MotionStaticThr.cpp
 *
 *  Created on: Jul 19, 2012
 *      Author: sourya
 *
 *      this file contains motion detection method which employs pixel level difference for static thresholding based decision
 *      the method was published (partially) in NCVPRIPG
 */

#include <motion_header.h>

/*
 * this function is for motion detection of the given input video file, using a frame by frame pixel comparison
 */
#ifdef SLEEP_APNEA_DETECT
void motion_detect_static_threshold(struct motion_detect_basic_struct *s, char* video_filename, double Thr_Ratio)
#else
void motion_detect_static_threshold(struct motion_detect_basic_struct *s, char* video_filename)
#endif
{
	CvCapture* cv_cap;
	struct Motion_Information *temp_motion_info;	// temporary pointer for motion information node

	double CUTOFF, ALPHA, THRESHOLD_RATIO;
	int tot, eq, neq;
	double matching_index;

	int i, j, k, state;
	int frame_count;
	IplImage* cap_frame = 0;
	IplImage* mod_frame = 0;
	IplImage* ref_frame = 0;
	IplImage* ext_frame = 0;

	// variables to store ongoing motion information
	double motion_start_time;
	double motion_end_time;
	int motion_start_frame_count;
	int motion_end_frame_count;

	// difference threshold between ongoing motion information and the previous one
	// int VIDEO_MOTION_TIME_DIFF_THR = 3;	// 3 sec is the maximum time difference

    // design the motion detection constant thresholds
    CUTOFF = 85;	//30;
    ALPHA = .1; //.05;

#ifdef SLEEP_APNEA_DETECT
    THRESHOLD_RATIO = Thr_Ratio;
#else
    THRESHOLD_RATIO = 0.03;		// 0.05	//.01;	//.002;
#endif

	// capture the video file - global structure
	cv_cap = cvCaptureFromAVI(video_filename);

	// read the first frame
	cap_frame = cvQueryFrame(cv_cap);

	// resize frame structure
	//IplImage *resize = cvCreateImage(cvSize(cap_frame->width, cap_frame->height),8,3);

	mod_frame = cvCreateImage(cvSize(cap_frame->width,cap_frame->height),IPL_DEPTH_8U, 3);
	ref_frame = cvCreateImage(cvSize(cap_frame->width,cap_frame->height),IPL_DEPTH_8U, 1);
	ext_frame = cvCreateImage(cvSize(cap_frame->width,cap_frame->height),IPL_DEPTH_8U, 1);

	mod_frame->origin = cap_frame->origin;
	ref_frame->origin = cap_frame->origin;
	ext_frame->origin = cap_frame->origin;

	cvCopy(cap_frame, mod_frame, 0);

    // motion defining variables
    state = 0;

    // currently the box draw and the boundary initializations are not used
    // we loop through the entire frames
    // frame counter is increased by 5 - half of the frame rate of the recorded video
    for(frame_count = 0; frame_count < (s->tail_video_info->no_of_frames - 2); frame_count += 5)	// there was originally <= operator
	{
		cvCopy(ext_frame, ref_frame, 0);

		// at first we have to seek to the target frame
		cvSetCaptureProperty(cv_cap, CV_CAP_PROP_POS_FRAMES, (frame_count+2));

		// now capture the target frame
		cap_frame = cvQueryFrame(cv_cap);

		// if there is a problem in current frame capture then continue with the next iteration of the loop
		if (!cap_frame)
			continue;

		// compare the current frame vs the background reference frame pixel by pixel comparison
		for(j = s->init_x; j <= s->final_x; j++)		//for(j = 0; j < cap_frame->width; j++)
		{
			for(i = s->init_y; i <= s->final_y; i++)		//for(i = 0; i < cap_frame->height; i++)
			{
				tot = 0;
                for(k = 1; k<3; k++)
				{
                    tot = tot + abs((mod_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + k]
							- (cap_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + k]);
                }
                // if the difference is greater than cutoff then we store the
				// pixel as black in the corresponding stored binary image
				if ((tot/3) > CUTOFF)
				{
					((uchar *)(ext_frame->imageData + i*cap_frame->width))[j] = 255;
				}
                else
				{
					// white pixel
					((uchar *)(ext_frame->imageData + i*cap_frame->width))[j] = 0;

					// update the background model
                    for(k = 1; k < 3; k++)
					{
						(mod_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + k] =
							(1-ALPHA) * (mod_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + k]
							+ ALPHA * (cap_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + k];
					}
                }
			}	// frame height loop
		}	// frame width loop

		// if current frame has significant difference corresponding to the background
		// reference frame then update the state variable

		eq = 0;
		neq = 0;
		for(j = s->init_x; j <= s->final_x; j++)		//for(j = 0; j < cap_frame->width; j++)
		{
			for(i = s->init_y; i <= s->final_y; i++)		//for(i = 0; i < cap_frame->height; i++)
			{
				if ((ref_frame->imageData + i*cap_frame->width)[j] != (ext_frame->imageData + i*cap_frame->width)[j])
					eq++;
                else
					neq++;
			}
		}
		if (neq != 0)
			matching_index = (double)((double)eq /(double) neq);
		else
			matching_index = 0;

		if (matching_index > THRESHOLD_RATIO)	// there is a motion because pixel difference is greater than the threshold
		{
			if (state == 0)
			{
				// motion is started
				state = 1;

				// note down the motion start frame and the start time
				motion_start_frame_count = frame_count;
				motion_start_time = frame_count / s->tail_video_info->fps;
			}
            else if (state == 1)
			{
				// motion is continued
			}
		}
        else	// there is no motion
		{
			if (state == 1)		// if there was ongoing motion previously
			{
				// motion is stopped
				state = 0;

				// note down the motion end frame and the end time
				motion_end_frame_count = frame_count;
				motion_end_time = frame_count / s->tail_video_info->fps;

# if 0
				// if current motion start time is very close to the last motion end time (the difference
				// is less than a predefined time threshold) then merge the current motion information
				// to the last one
				if ((tail_video_info->motion_count > 0) &&
					((motion_start_time - tail_video_info->tail_motion_info->motion_end_time) <= VIDEO_MOTION_TIME_DIFF_THR))
				{
					// update the last stored motion information (end frame count and end time)
					tail_video_info->tail_motion_info->motion_end_frame_count = motion_end_frame_count;
					tail_video_info->tail_motion_info->motion_end_time = motion_end_time;
				}
				else
				{
# endif
					// there should be a new motion information entry
					// create a motion information node which will be associated with current video file
					temp_motion_info = (struct Motion_Information *)malloc(sizeof(struct Motion_Information));
					if (temp_motion_info == NULL)
					{
						printf("Insufficient memory - no new motion node alloc");
						return;
					}

					temp_motion_info->next = NULL;
					temp_motion_info->prev = NULL;
					if (s->tail_video_info->motion_count == 0)	//(tail_video_info->head_motion_info == NULL && tail_video_info->tail_motion_info == NULL)	//  this is the first node
					{
						s->tail_video_info->head_motion_info = s->tail_video_info->tail_motion_info = temp_motion_info;		// assign pointer
					}
					else
					{
						temp_motion_info->prev = s->tail_video_info->tail_motion_info;
						s->tail_video_info->tail_motion_info->next = temp_motion_info;
						s->tail_video_info->tail_motion_info = temp_motion_info;			// assign pointer
					}

					// assign the fields onto this motion node structure
					s->tail_video_info->tail_motion_info->motion_start_frame_count = motion_start_frame_count;
					s->tail_video_info->tail_motion_info->motion_start_time = motion_start_time;
					s->tail_video_info->tail_motion_info->motion_end_frame_count = motion_end_frame_count;
					s->tail_video_info->tail_motion_info->motion_end_time = motion_end_time;

					// increment the motion node counter
					s->tail_video_info->motion_count++;
# if 0
				}
# endif

# if 0
				// update the reference frames
                ref_frame = cvCreateImage(cvSize(cap_frame->width,cap_frame->height),IPL_DEPTH_8U, 1);
				ext_frame = cvCreateImage(cvSize(cap_frame->width,cap_frame->height),IPL_DEPTH_8U, 1);
				ref_frame->origin = cap_frame->origin;
				ext_frame->origin = cap_frame->origin;
# endif

				cvCopy(cap_frame, mod_frame, 0);
			}
		}
	} // end of frame by frame comparison loop

	// release the capture
	cvReleaseCapture(&cv_cap);

	cvReleaseImage(&mod_frame);
	cvReleaseImage(&ref_frame);
	cvReleaseImage(&ext_frame);

	// if there was ongoing motion then we note down the motion end mark
	if (state == 1)
	{
		// motion is stopped
		state = 0;

		// note down the motion end frame and the end time
		motion_end_frame_count = (int)(s->tail_video_info->no_of_frames - 2);
		motion_end_time = (s->tail_video_info->no_of_frames - 2) / s->tail_video_info->fps;

# if 0
		// if current motion start time is very close to the last motion end time (the difference
		// is less than a predefined time threshold) then merge the current motion information
		// to the last one
		if ((tail_video_info->motion_count > 0) &&
			((motion_start_time - tail_video_info->tail_motion_info->motion_end_time) <= VIDEO_MOTION_TIME_DIFF_THR))
		{
			// update the last stored motion information (end frame count and end time)
			tail_video_info->tail_motion_info->motion_end_frame_count = motion_end_frame_count;
			tail_video_info->tail_motion_info->motion_end_time = motion_end_time;
		}
		else
		{
# endif
			// there should be a new motion information entry
			// create a motion information node which will be associated with current video file
			temp_motion_info = (struct Motion_Information *)malloc(sizeof(struct Motion_Information));
			if (temp_motion_info == NULL)
			{
				printf("Insufficient memory - no new motion node alloc");
				return;
			}

			temp_motion_info->next = NULL;
			temp_motion_info->prev = NULL;
			if (s->tail_video_info->motion_count == 0)	//(tail_video_info->head_motion_info == NULL && tail_video_info->tail_motion_info == NULL)	//  this is the first node
			{
				s->tail_video_info->head_motion_info = s->tail_video_info->tail_motion_info = temp_motion_info;		// assign pointer
			}
			else
			{
				temp_motion_info->prev = s->tail_video_info->tail_motion_info;
				s->tail_video_info->tail_motion_info->next = temp_motion_info;
				s->tail_video_info->tail_motion_info = temp_motion_info;			// assign pointer
			}

			// assign the fields onto this motion node structure
			s->tail_video_info->tail_motion_info->motion_start_frame_count = motion_start_frame_count;
			s->tail_video_info->tail_motion_info->motion_start_time = motion_start_time;
			s->tail_video_info->tail_motion_info->motion_end_frame_count = motion_end_frame_count;
			s->tail_video_info->tail_motion_info->motion_end_time = motion_end_time;

			// increment the motion node counter
			s->tail_video_info->motion_count++;
# if 0
		}
# endif
	}

}	// end function


