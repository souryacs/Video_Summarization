/* this project is meant for implementing video summarization technique
 * based on selective motion detection
 *
 * this is the main executable file of the project
 *
 * author : Sourya Bhattacharyya
 * version : 5.1
 * 27.04.2012 - basic main function and correlogram, compute code
 * 30.06.2012 - added code for computing the dynamic foreground and motion detection based on dynamic thresholding
 * 17.07.2012 - added a new main function for motion vector calculation
 * 18.07.2012 - added another module to incorporate the calculation of motion history and motion gradient image
 * 22.07.2012 - defined different motion types and also generated the feature vectors for classification of different types of motion
 * 04.08.2012 - added optical flow based motion computation and related gradient calculation methods
 * 20.01.2013 - added some define statements and clarifications of video file name conventions
 */

#include <motion_header.h>

/*
 * this statement enables to mention the starting video file where the motion detection will start (for example, 0 means
 * filename nrva0000.avi, 1 means nrva0001.avi etc
 */
#define VIDEO_FILE_START_NUMBER 0	//2

/*
 * this statement defines the default value of frame rate to be used (unless otherwise specified)
 */
#if !defined(SLEEP_APNEA_DETECT)
#define FPS_VALUE 10	//25
#endif

struct motion_detect_basic_struct mdbs;

/* global variable indicating the name of the text file containing the video motion information
 *
 * if the video file is generated from optical flow based motion detection method
 * then the name is "video_motion_info_derived_staticthr_fullwindow.txt"  (for full window based motion detection, without enclosing with a rectangle box)
 *					"video_motion_derived_staticthr_box.txt" (for enclosing with a rectangle box)
 * if the motion is detected by calculating dynamic thresholds based on statistical computation,
 * 			then the video file is "video_motion_derived_dynamic_backgrnd_box.txt"
 * if the motive is to generate ground truth labels of the video segments via using frame by frame monitoring
 * 			then the name is "complete_label_set_ground_truth.txt"
 * if the text file is annotated and we have to extract the features for corresponding segments
 * 			then the file name is video_motion_manual_annotation.txt
 */

#if !defined(SLEEP_APNEA_DETECT)
	char vid_txt_fname[FILENAME_STRING_LEN] = "video_motion_manual_annotation.txt";
#endif

/*
 * this segment is used for computation of threshold based motion detection
 * or for dynamic foreground calculation
 * to minimize the effects caused by external movements
 * however it is not required for motion history and gradient modeling
 */

# if defined(DYNAMIC_FOREGROUND_MOTION_COMPUTE) || defined(PIXEL_DIFFERENCE_MOTION_MEASURE)

CvRect box;
bool drawing_box = false;

/*
 * this function draws the box
 */
void draw_box( IplImage* img, CvRect rect )
{
	cvRectangle(img, cvPoint(box.x, box.y), cvPoint((box.x + box.width), (box.y + box.height)), cvScalar(0xff,0x00,0x00));
}

/*
 * mouse handler to draw a box based on mouse movement
 */
void mouseHandler(int event, int x, int y, int flags, void *param)
{
    IplImage* image = (IplImage*) param;

	switch( event )
	{
		case CV_EVENT_MOUSEMOVE:
			if(drawing_box)
			{
				box.width = x - box.x;
				box.height = y - box.y;
			}
			break;

		case CV_EVENT_LBUTTONDOWN:
			drawing_box = true;
			box = cvRect( x, y, 0, 0 );
			break;

		case CV_EVENT_LBUTTONUP:
			drawing_box = false;
			if( box.width < 0 )
			{
				box.x += box.width;
				box.width *= -1;
			}
			if( box.height < 0 )
			{
				box.y += box.height;
				box.height *= -1;
			}
			draw_box(image, box);
			break;
	}
}

# endif		/* # if defined(DYNAMIC_FOREGROUND_MOTION_COMPUTE) || defined(PIXEL_DIFFERENCE_MOTION_MEASURE) */

/*
 * main function for program execution
 * modification  --27.05.2013 - sourya
 * for static threshold based motion detection, in the command line arguments, we supply following parameters
 * 1. Input video file name
 * 2. Frame rate information of the input video
 * 3. THRESHOLD_RATIO - is the ratio of motion pixels to the total input pixels, based on which, motion detection decision is taken
 */
int main(int argc, char* argv[])
{
	// in the command line argument, user will supply the directory containing the video files
	// that directory will be used to traverse the recorded video files
	char temp_vid_index_str[10];
	struct Video_Information *temp_video_info;	// temporary pointer to a video frame information
	char vid_file_name[FILENAME_STRING_LEN];	// individual video file name

	// this file contains the motion detection output
	char vid_text_file_name[FILENAME_STRING_LEN];

	// opencv defined structure
	CvCapture *capture;
	IplImage* temp_cap_frame = 0;

#if defined(SLEEP_APNEA_DETECT)
	double inp_fps_val;
	double Thr_Ratio;
#endif

	/*
	 * these variables are for dynamic foreground computation based motion detection method
	 */
# ifdef DYNAMIC_FOREGROUND_MOTION_COMPUTE

	// stores the frames for computing the temporal difference
	IplImage* first_frame;
	IplImage* middle_frame;
	IplImage* last_frame;

	// computes the background reference frame
	IplImage* background_frame;

	// detected foreground image used to generate the final foreground picture
	IplImage* det_for_img;

	int i,j,k;
# endif

	// initialize the head and tail video file pointers
	mdbs.no_of_video_files = VIDEO_FILE_START_NUMBER;
	mdbs.head_video_info = NULL;
	mdbs.tail_video_info = NULL;

	// counts the no of frames within the video file
	int frame_count = 0;

	// if there is no input video file directory present then the code will exit
#if defined(SLEEP_APNEA_DETECT)
	if (argc < 5)
	{
		fprintf(stderr, "Kindly specify the input video filename, input video frame rate, thresholding ratio, and output text file information --  which will be used for motion detection");
		fprintf(stderr, "Command line argument is missing");
		return 0;
	}
#else
	if (argc < 2)
	{
		fprintf(stderr, "Kindly specify the input video filename which will be used for motion detection");
		fprintf(stderr, "Command line argument is missing");
		return 0;
	}
#endif


#if !defined(SLEEP_APNEA_DETECT)
	// directory containing the video files
	mdbs.directory_name[0] = '\0';
	//strcat(mdbs.directory_name, "/home/sourya/Documents/video_eeg_motion_check_data/SUTRADHAR_BABY OF_S@20110901_151228/Patient1t1/");
	//strcat(mdbs.directory_name, "/media/FreeAgent Drive/Video_EEG_data/18_03_09_2011/SANTRA_BABY OF_S@20110823_144101/Patient1t1/");
	strcat(mdbs.directory_name, argv[1]);

	// construct the motion information containing file name (if it exists)
	vid_text_file_name[0] = '\0';
	strcat(vid_text_file_name, mdbs.directory_name);
	strcat(vid_text_file_name, vid_txt_fname);

#endif

#if defined(SLEEP_APNEA_DETECT)
	vid_file_name[0] = '\0';		// input video file for processing
	strcat(vid_file_name, argv[1]);
	mdbs.directory_name[0] = '\0';
	strncat(mdbs.directory_name, vid_file_name, (int)(strrchr(vid_file_name, '/') - vid_file_name + 2));
	inp_fps_val = atof(argv[2]);	// convert input frame rate
	Thr_Ratio = atof(argv[3]);		// threshold ratio for motion detection
	vid_text_file_name[0] = '\0';		// output text file for motion detection output write
	strcat(vid_text_file_name, argv[4]);
	printf("\n input video: %s dir: %s fps: %lf Thr : %lf vid_txt : %s ", vid_file_name, mdbs.directory_name, inp_fps_val, Thr_Ratio, vid_text_file_name);
#endif

	/*
	 * this while loop will execute unless we want to calculate the motion history based features
	 * on the already decided motion segments
	 */
# if !defined(MEI_MHI_MOTION_MEASURE) && !defined(OPTICAL_FLOW_MOTION_VECTOR) && !defined(GENERATELABEL)

	// loop to read individual video file information
	while (1)
	{
#if !defined(SLEEP_APNEA_DETECT)
		// form the video file name string
		vid_file_name[0] = '\0';
		strcat(vid_file_name, mdbs.directory_name);
		decimal_to_hex(mdbs.no_of_video_files, &temp_vid_index_str[0]);
		strcat(vid_file_name, "nrva");
		strcat(vid_file_name, temp_vid_index_str);
		strcat(vid_file_name, ".avi");
#else


#endif
		// check if the file is found
		if (access(vid_file_name, 0) == 0)	{

			// if the file exists then capture its header by opencv standard function
			// at first we grab one frame
			capture = cvCaptureFromAVI(vid_file_name);

			// get the first frame
			temp_cap_frame = cvQueryFrame(capture);
			frame_count++;

			if (capture) {
				printf("\n video file name : %s ", vid_file_name);

				// allocate one node of list which will contain information of current video file
				temp_video_info = (struct Video_Information *) malloc(sizeof(struct Video_Information));
				if (temp_video_info == NULL) {
					printf("Insufficient memory - no new video node alloc");
					return 0;
				}
				temp_video_info->next = NULL;
				temp_video_info->prev = NULL;
				if (mdbs.head_video_info == NULL && mdbs.tail_video_info == NULL) {
					//  this is the first node
					mdbs.head_video_info = mdbs.tail_video_info = temp_video_info; // assign pointer
				} else {
					temp_video_info->prev = mdbs.tail_video_info;
					mdbs.tail_video_info->next = temp_video_info;
					mdbs.tail_video_info = temp_video_info; // assign pointer
				}

				// fill the parameters for current video node
				mdbs.tail_video_info->no_of_frames = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT); // no of frames

				//-------------------------------------------
				/*
				 * FPS get property does not work in linux - fps is always returned as zero
				 * so the code is hard coded as 10 - sourya
				 */
#if !defined(PIXEL_DIFFERENCE_MOTION_MEASURE)
				mdbs.tail_video_info->fps = FPS_VALUE;		//(int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS); // fps
#else
				mdbs.tail_video_info->fps = inp_fps_val;
#endif
				if (mdbs.tail_video_info->fps != 0)	{
					mdbs.tail_video_info->duration = (mdbs.tail_video_info->no_of_frames / mdbs.tail_video_info->fps); //duration
				}
				else {
					mdbs.tail_video_info->duration = 0;
				}

				//-------------------------------------------
				// does not work in linux
				//mdbs.tail_video_info->frame_height = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
				//mdbs.tail_video_info->frame_width = cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);

				// works in linux
				mdbs.tail_video_info->frame_height = temp_cap_frame->height;
				mdbs.tail_video_info->frame_width = temp_cap_frame->width;
				//-------------------------------------------

				mdbs.tail_video_info->motion_count = 0; //initialization
				mdbs.tail_video_info->head_motion_info = NULL;
				mdbs.tail_video_info->tail_motion_info = NULL;
				//strcpy(mdbs.tail_video_info->video_filename, temp1);	// current video file name

				// write the video related information
				printf( "\n %d ", mdbs.no_of_video_files);
				//printf( "%s\n", mdbs.tail_video_info->video_filename);
				printf( "\t %lf", mdbs.tail_video_info->fps);
				printf( "\t %lf", mdbs.tail_video_info->duration);
				printf( "\t %lf", mdbs.tail_video_info->no_of_frames);
				printf( "\t %lf", mdbs.tail_video_info->frame_height);
				printf( "\t %lf", mdbs.tail_video_info->frame_width);
				printf( "\t %d", mdbs.tail_video_info->motion_count);

				// add - sourya
				//printf("\n\n \t motion start \t motion end");

# if 0
				printf( "%d %d %d %d %d %d \n", mdbs.tail_video_info->video_file_create_time.GetDay(),
							mdbs.tail_video_info->video_file_create_time.GetMonth(),
							mdbs.tail_video_info->video_file_create_time.GetYear(),
							mdbs.tail_video_info->video_file_create_time.GetHour(),
							mdbs.tail_video_info->video_file_create_time.GetMinute(),
							mdbs.tail_video_info->video_file_create_time.GetSecond());
				printf( "%lf\n", mdbs.tail_video_info->curr_video_file_start_time);
				printf( "%lf\n", mdbs.tail_video_info->curr_video_file_end_time);
# endif


				/*
				 * 	this code calls the rectangle box draw surrounding the patient
				 * 	this is for the first video file only
				 */
				if (mdbs.no_of_video_files == VIDEO_FILE_START_NUMBER /* 0 */)
				{
					/***********************************/
					/*
					 * this segment is used for computation of threshold based motion detection
					 * or for dynamic foreground calculation
					 * to minimize the effects caused by external movements
					 * however it is not required for motion history and gradient modeling
					 */
# if defined(DYNAMIC_FOREGROUND_MOTION_COMPUTE) || defined(PIXEL_DIFFERENCE_MOTION_MEASURE)

					// this segment draws a window box for patient identification
					box = cvRect(-1,-1,0,0);
					IplImage* temp = cvCloneImage(temp_cap_frame);

					// Create windows
					cvNamedWindow("Video: Draw rectangle and then press c", CV_WINDOW_AUTOSIZE);
					cvSetMouseCallback( "Video: Draw rectangle and then press c", mouseHandler, (void*)temp_cap_frame);
					while(1)
					{
						cvCopyImage(temp_cap_frame, temp);
						if(drawing_box)
							draw_box(temp, box);
						cvShowImage("Video: Draw rectangle and then press c", temp);
						char tt;
						tt = cvWaitKey(15);
						if(tt == 'c' || tt == 'C') //press 'c' character to exit the loop	//27 is the ascii value
						{
							//printf("\n\n *** video box has been finalized ****");
							break;
						}
					}
					//printf("\n box.x = %d, box.y = %d, box.x+box.width = %d, box.y+box.height = %d", box.x, box.y, box.x+box.width, box.y+box.height);
					mdbs.init_x = box.x ;
					mdbs.init_y = box.y ;
					mdbs.final_x = box.x + box.width;
					mdbs.final_y = box.y + box.height;
					if (mdbs.init_x > mdbs.final_x)
					{
						mdbs.init_x = mdbs.final_x;
						mdbs.final_x = box.x;
					}
					if (mdbs.init_y > mdbs.final_y)
					{
						mdbs.init_y = mdbs.final_y;
						mdbs.final_y = box.y;
					}
					cvDestroyWindow("Video: Draw rectangle and then press c");
					// end of window box segment
# else

					/*
					 * this module is required for motion history and gradient modeling
					 * should be commented when motion detection based on thresholding or dynamic foreground modeling is used
					 * because there the external movements should be minimized as much as possible
					 * leading to the drawing of rectangular box
					 */

					// added - sourya
					mdbs.init_x = 0;
					mdbs.init_y = 0;
					mdbs.final_x = mdbs.tail_video_info->frame_width;
					mdbs.final_y = mdbs.tail_video_info->frame_height;
# endif
					/***********************************/

					/*
					 * this part is used for motion detection based on dynamic background modeling
					 * there it is required to model and update the background reference frame, the current frame and the next frame
					 * in order to compute the temporal difference
					 */
# ifdef DYNAMIC_FOREGROUND_MOTION_COMPUTE
					// allocate the image structures required to contain the frames
					first_frame = cvCreateImage(cvSize(temp_cap_frame->width, temp_cap_frame->height),IPL_DEPTH_8U, 3);
					middle_frame = cvCreateImage(cvSize(temp_cap_frame->width, temp_cap_frame->height),IPL_DEPTH_8U, 3);
					last_frame = cvCreateImage(cvSize(temp_cap_frame->width, temp_cap_frame->height),IPL_DEPTH_8U, 3);

					first_frame->origin = temp_cap_frame->origin;
					middle_frame->origin = temp_cap_frame->origin;
					last_frame->origin = temp_cap_frame->origin;

					// allocate the background reference frame
					background_frame = cvCreateImage(cvSize(temp_cap_frame->width, temp_cap_frame->height),IPL_DEPTH_8U, 3);
					background_frame->origin = temp_cap_frame->origin;

					// allocate the detected foreground frame
					det_for_img = cvCreateImage(cvSize(temp_cap_frame->width, temp_cap_frame->height),IPL_DEPTH_8U, 1);
					for (i = 0; i < temp_cap_frame->height; i++)	// fill all the values with 0
					{
						for (j = 0; j < temp_cap_frame->width; j++)
						{
							(det_for_img->imageData + i*det_for_img->widthStep)[j] = 0;
						}
					}
					det_for_img->origin = temp_cap_frame->origin;
# endif

					/***********************************/
				}	// for the very first video file

				/*******************************************/
				/*
				 * for use with Chiranjivi paper
				 * this part is used for correlogram computation and motion detection
				 */
# ifdef CORRELOGRAM_MOTION_MEASURE
				/*
				 * these functions are responsible for allocating the structures for processing individual frames
				 * for the very first video file input and the first frame
				 */
				if ((mdbs.no_of_video_files == 0) && (frame_count == 1))
				{
					AllocateAllStructures(&mdbs);
				}
# endif
				/*******************************************/

				/*
				 * the outer for loop is used for motion detection based on any of the three algorithms
				 * either the dynamic foreground modeling
				 * or the correlogram computation based method
				 * or the optical flow based motion vector computation
				 */

# if defined(CORRELOGRAM_MOTION_MEASURE) || defined(DYNAMIC_FOREGROUND_MOTION_COMPUTE)

				// this section will process frames (half of the frame rate)
				for(frame_count = 0; frame_count < (mdbs.tail_video_info->no_of_frames - 2); frame_count += 5)
				{
					// at first we have to seek to the target frame
					cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, (frame_count + 2));

					// now capture the target frame
					temp_cap_frame = cvQueryFrame(capture);

					// if there is a problem in current frame capture then continue with the next iteration of the loop
					if (!temp_cap_frame)
						continue;

					/*
					 * this part is used for motion computation based on dynamic foreground modeling
					 * reference, current and next frames are updated on each iteration
					 */

# ifdef DYNAMIC_FOREGROUND_MOTION_COMPUTE

					if (frame_count == 0)	// copy to first frame
						cvCopy(temp_cap_frame, first_frame, 0);
					else if (frame_count == 5)
						cvCopy(temp_cap_frame, middle_frame, 0);
					else if (frame_count == 10)
						cvCopy(temp_cap_frame, last_frame, 0);
					else
					{
						cvCopy(middle_frame, first_frame, 0);
						cvCopy(last_frame, middle_frame, 0);
						cvCopy(temp_cap_frame, last_frame, 0);
					}

					if (frame_count >= 10)
					{
						/*
						 * call the motion detection routine
						 * the routine is based on the paper
						 * A Novel Hybrid Motion Detection Algorithm Based On Dynamic Thresholding Segmentation
						 * by Peng Zhang, Tie-Yong Cao, Tao Zhu
						 */
						HybridMotionDetect(&mdbs, first_frame, middle_frame, last_frame, background_frame, frame_count, det_for_img);
					}

					/*
					 * the background reference frame also needs to be updated
					 * but for the current iteration (at time t) the previous background ref frame (at time t-1) needs to be used
					 * so the reference frame updation is to be done after the main function call
					 */
					if (frame_count == 0)	// first frame is the initial background reference
					{
						cvCopy(temp_cap_frame, background_frame, 0);
					}
					else
					{
						for (i = 0; i < temp_cap_frame->height; i++)
						{
							for (j = 0; j < temp_cap_frame->width; j++)
							{
								for (k = 1; k < 3; k++)
								{
									(background_frame->imageData + i*background_frame->widthStep)[j*background_frame->nChannels + k] =
											(1 - TH_ALPHA) * (background_frame->imageData + i*background_frame->widthStep)[j*background_frame->nChannels + k] +
												TH_ALPHA * (temp_cap_frame->imageData + i*temp_cap_frame->widthStep)[j*temp_cap_frame->nChannels + k];
								}
							}
						}
					}

# endif		/* DYNAMIC_FOREGROUND_MOTION_COMPUTE */

					/*******************************************/
					/*
					 * for use with Chiranjivi paper
					 * this part is used for correlogram computation and motion detection
					 */
# ifdef CORRELOGRAM_MOTION_MEASURE
					/*
					 * compute the correlogram for the individual pixels
					 */
					CorrelogramCompute(&mdbs, temp_cap_frame, frame_count);
# endif
					/*******************************************/

				}	// end frame loop
# endif

				/*******************************************/
				/*
				 * this part is used for computation of the motion detection method which uses static thresholding
				 * this method is the custom reference (earlier published in NCVPRIPG)
				 */

# ifdef PIXEL_DIFFERENCE_MOTION_MEASURE
				/*
				 * at first detect the motion segments
				 * motion information per video file will be detected
				 */
# ifdef SLEEP_APNEA_DETECT
				motion_detect_static_threshold(&mdbs, vid_file_name, Thr_Ratio);
# else
				motion_detect_static_threshold(&mdbs, vid_file_name);
# endif
# endif

# if defined(PIXEL_DIFFERENCE_MOTION_MEASURE) || defined(DYNAMIC_FOREGROUND_MOTION_COMPUTE)
				/*
				 * now write the motion information on a file
				 * motion information per video file will be written
				 */
				Write_Motion_Information(&mdbs, vid_text_file_name);
# endif

				/*******************************************/

				// after processing all the frames in the current video file
				// release the capture
				cvReleaseCapture(&capture);

				// increment the counter
				mdbs.no_of_video_files++;

			} // end if video file is captured
			else	{
				break;
			}
		}	// end if video file exists
		else	{
			// break from this while loop
			break;
		}
	}	// end while loop

# endif /* MEI_MHI_MOTION_MEASURE */

	/*
	 * this section is added
	 * debug for feature extraction
	 */
# if defined(MEI_MHI_MOTION_MEASURE) || defined(OPTICAL_FLOW_MOTION_VECTOR) || defined(GENERATELABEL)
	/*
	 * at first read from the text file all the motion information
	 */
	Read_Video_Motion_Information(&mdbs, vid_text_file_name);
# endif

	/*
	 * now for individual motion instance (video file, start frame and end frame), pass the information
	 * to the motion history image function
	 * the following function serves as a wrapper
	 *
	 */
# ifdef OPTICAL_FLOW_MOTION_VECTOR
	/*
	 * this function it processes the motion instances one at a time and then passes individual information to the
	 * optical flow based MV computing function
	 */
	Optical_Flow_MV_wrapper(&mdbs);
# endif

# ifdef MEI_MHI_MOTION_MEASURE
	/*
	 * this function it processes the motion instances one at a time and then passes individual information to the
	 * motion history image computing function
	 */
	Motion_Info_to_History_Img_Wrapper(&mdbs);
# endif

#ifdef GENERATELABEL
	/*
	 * this function calls for manual annotation of the video segments which have been automatically identified
	 * as a motion segment by optical flow or dynamic background based motion detection technique
	 */
	ManualAnnotateSegments(&mdbs);
# endif
	/*******************************************/
	/*
	 * for use with Chiranjivi paper
	 * this part is used for correlogram computation and motion detection
	 */
# ifdef CORRELOGRAM_MOTION_MEASURE
	// before exiting, free all the structures that are allocated during computation of the fuzzy correlogram
	FreeAllocatedStructures(&mdbs);
# endif
	/*******************************************/

	/*
	 * these variables are for dynamic foreground computation based motion detection method
	 */
# ifdef DYNAMIC_FOREGROUND_MOTION_COMPUTE
	// free the allocated image structures
	cvReleaseImage(&first_frame);
	cvReleaseImage(&middle_frame);
	cvReleaseImage(&last_frame);
	cvReleaseImage(&background_frame);
	cvReleaseImage(&det_for_img);
# endif

	return 0;
}	// end main function

