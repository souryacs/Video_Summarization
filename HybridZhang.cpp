/*
 * HybridZhang.cpp
 *
 *  Created on: June 19, 2012
 *      Author: sourya
 */

#include <motion_header.h>

// structures used in the paper implementation Zhang et.al.
bool **temporal_foreground;
bool **temporal_foreground_mask;
bool **spatial_foreground;
bool **final_detected_foreground;
double **temp_diff_image;
double **spat_diff_image;
double **temp_diff_image_plus1;

/*
 * frees the allocated structure
 * paper Zhang et. al.
 */
void FreeStruct(int no_of_pixels_x, int no_of_pixels_y)
{
	int i;
	for (i = 0; i < no_of_pixels_y; i++)
		free(temp_diff_image[i]);
	free(temp_diff_image);

	for (i = 0; i < no_of_pixels_y; i++)
		free(temp_diff_image_plus1[i]);
	free(temp_diff_image_plus1);

	for (i = 0; i < no_of_pixels_y; i++)
		free(spat_diff_image[i]);
	free(spat_diff_image);

	for (i = 0; i < no_of_pixels_y; i++)
		free(temporal_foreground[i]);
	free(temporal_foreground);

	for (i = 0; i < no_of_pixels_y; i++)
		free(temporal_foreground_mask[i]);
	free(temporal_foreground_mask);

	for (i = 0; i < no_of_pixels_y; i++)
		free(spatial_foreground[i]);
	free(spatial_foreground);

	for (i = 0; i < no_of_pixels_y; i++)
		free(final_detected_foreground[i]);
	free(final_detected_foreground);

	return;
}	// end function


/*
 * write the allocation routine of the corresponding structures
 */
void AllocateStruct(int no_of_pixels_x, int no_of_pixels_y)
{
	int i;

	// allocate temporal difference image (middle and first frame)
	temp_diff_image = (double**)calloc(no_of_pixels_y, sizeof(double *));
	for (i = 0; i < no_of_pixels_y; i++)
		temp_diff_image[i] = (double *)calloc(no_of_pixels_x, sizeof(double));

	// allocate temporary difference image (last and middle frame)
	temp_diff_image_plus1 = (double**)calloc(no_of_pixels_y, sizeof(double *));
	for (i = 0; i < no_of_pixels_y; i++)
		temp_diff_image_plus1[i] = (double *)calloc(no_of_pixels_x, sizeof(double));

	// allocate spatial difference image
	spat_diff_image = (double**)calloc(no_of_pixels_y, sizeof(double *));
	for (i = 0; i < no_of_pixels_y; i++)
		spat_diff_image[i] = (double *)calloc(no_of_pixels_x, sizeof(double));

	// allocate temporal foreground structure
	temporal_foreground = (bool**)calloc(no_of_pixels_y, sizeof(bool *));
	for (i = 0; i < no_of_pixels_y; i++)
		temporal_foreground[i] = (bool *)calloc(no_of_pixels_x, sizeof(bool));

	// allocate temporal foreground mask structure
	temporal_foreground_mask = (bool**)calloc(no_of_pixels_y, sizeof(bool *));
	for (i = 0; i < no_of_pixels_y; i++)
		temporal_foreground_mask[i] = (bool *)calloc(no_of_pixels_x, sizeof(bool));

	// allocate spatial foreground
	spatial_foreground = (bool**)calloc(no_of_pixels_y, sizeof(bool *));
	for (i = 0; i < no_of_pixels_y; i++)
		spatial_foreground[i] = (bool *)calloc(no_of_pixels_x, sizeof(bool));

	// final detected foreground
	final_detected_foreground = (bool**)calloc(no_of_pixels_y, sizeof(bool *));
	for (i = 0; i < no_of_pixels_y; i++)
		final_detected_foreground[i] = (bool *)calloc(no_of_pixels_x, sizeof(bool));

	return;
}	// end function


double calc_mean(double **img, int init_x, int init_y, int final_x, int final_y)
{
	int i, j;
	double mean;
	mean = 0;
	for (j = init_y; j <= final_y; j++)
	{
		for (i = init_x; i <= final_x; i++)
		{
			mean += img[j - init_y][i - init_x];
		}
	}
	mean /= (final_x - init_x + 1) * (final_y - init_y + 1);
	return mean;
}

double calc_std(double **img, int init_x, int init_y, int final_x, int final_y, double mean)
{
	int i, j;
	double std;
	std = 0;
	for (j = init_y; j <= final_y; j++)
	{
		for (i = init_x; i <= final_x; i++)
		{
			std += pow((img[j - init_y][i - init_x] - mean), 2);
		}
	}
	std = sqrt(std / ((final_x - init_x + 1) * (final_y - init_y + 1)));
	return std;
}


void HybridMotionDetect(struct motion_detect_basic_struct *s, IplImage* first_frame, IplImage* middle_frame,
						IplImage* last_frame, IplImage* backgrnd_frame, int frame_count, IplImage* det_for_img)
{
	struct Motion_Information *temp_motion_info;	// temporary pointer for motion information node
	int no_of_pixels_x, no_of_pixels_y;
	int i, j, k;

	double temp_diff_img_mean, temp_diff_img_std;
	double temp_diff_image_plus1_mean, temp_diff_image_plus1_std;
	double spat_diff_img_mean, spat_diff_img_std;

	bool r_t, r_tplus1;
	double tot1, tot2, tot3;

	static int motion_ongoing = 0;

	// variables to store ongoing motion information
	double motion_start_time;
	double motion_end_time;
	static int motion_start_frame_count = 0;
	static int motion_end_frame_count = 0;

	// these variables are used for defining the temporal foreground mask
	int minx, miny, maxx, maxy;

	/*
	 * following condition ensures that for a new video file to be checked, the motion signifying variable becomes reset
	 */
	if (frame_count == 10)
		motion_ongoing = 0;

	// allocate the structures used for calculating temporal difference
	// and the motion detection
	no_of_pixels_x = (s->final_x - s->init_x + 1);
	no_of_pixels_y = (s->final_y - s->init_y + 1);

	// allocate the dynamic memory
	AllocateStruct(no_of_pixels_x, no_of_pixels_y);

	// define the temporal foreground mask  boundaries (initial)
	minx = (s->final_x - s->init_x);
	maxx = 0;
	miny = (s->final_y - s->init_y);
	maxy = 0;

	/******************************/
	// add  - sourya
# if 0
	double white_pix = 0, black_pix = 0;
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			if ((det_for_img->imageData + i*det_for_img->widthStep)[j] == 0)
				black_pix++;
			else
				white_pix++;
		}
	}

	printf("\n\n  ---- iteration -- white pix : %lf  black pix : %lf ", white_pix, black_pix);

# endif
	/******************************/

	// now fill the values of temporal difference images
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			// accumulate the intensity difference
			tot1 = 0;	tot2 = 0;	tot3 = 0;
			for (k = 1; k < 3; k++)
			{
				tot1 += (first_frame->imageData + i*first_frame->widthStep)[j*first_frame->nChannels + k];
				tot2 += (middle_frame->imageData + i*middle_frame->widthStep)[j*middle_frame->nChannels + k];
				tot3 += (last_frame->imageData + i*last_frame->widthStep)[j*last_frame->nChannels + k];
			}
			// fill the temporary difference images
			temp_diff_image[i - s->init_y][j - s->init_x] = fabs(tot2 - tot1);
			temp_diff_image_plus1[i - s->init_y][j - s->init_x] = fabs(tot3 - tot2);
		}
	}

	// calculate the mean and standard deviation of the difference images
	temp_diff_img_mean = calc_mean(temp_diff_image, s->init_x, s->init_y, s->final_x, s->final_y);
	temp_diff_img_std = calc_std(temp_diff_image, s->init_x, s->init_y, s->final_x, s->final_y, temp_diff_img_mean);

	temp_diff_image_plus1_mean = calc_mean(temp_diff_image_plus1, s->init_x, s->init_y, s->final_x, s->final_y);
	temp_diff_image_plus1_std = calc_std(temp_diff_image_plus1, s->init_x, s->init_y, s->final_x, s->final_y, temp_diff_image_plus1_mean);

	// now calculate the temporal foreground
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			r_t = (fabs(temp_diff_image[i - s->init_y][j - s->init_x] - temp_diff_img_mean) > (TH_K * temp_diff_img_std)) ? 1 : 0;
			r_tplus1 = (fabs(temp_diff_image_plus1[i - s->init_y][j - s->init_x] - temp_diff_image_plus1_mean) > (TH_K * temp_diff_image_plus1_std)) ? 1 : 0;

			// fill the value according to the AND operation
			temporal_foreground[i - s->init_y][j - s->init_x] = (r_t && r_tplus1);
			if (temporal_foreground[i - s->init_y][j - s->init_x] == 1)
			{
				// update the minimum and maximum x and y boundaries
				// it'll be needed later to define the temporal foreground mask
				minx = (minx > (j - s->init_x)) ? (j - s->init_x) : minx;
				maxx = (maxx < (j - s->init_x)) ? (j - s->init_x) : maxx;
				miny = (miny > (i - s->init_y)) ? (i - s->init_y) : miny;
				maxy = (maxy < (i - s->init_y)) ? (i - s->init_y) : maxy;
			}
		}
	}

	// create the temporal foreground mask
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			if (((j - s->init_x) >= minx) && ((j - s->init_x) <= maxx) && ((i - s->init_y) >= miny) && ((i - s->init_y) <= maxy))
				temporal_foreground_mask[i - s->init_y][j - s->init_x] = 1;
			else
				temporal_foreground_mask[i - s->init_y][j - s->init_x] = 0;
		}
	}

	// now fill the values of spatial foreground
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			// accumulate the intensity difference
			tot1 = 0;	tot2 = 0;
			for (k = 1; k < 3; k++)
			{
				tot1 += (middle_frame->imageData + i*middle_frame->widthStep)[j*middle_frame->nChannels + k];
				tot2 += (backgrnd_frame->imageData + i*backgrnd_frame->widthStep)[j*backgrnd_frame->nChannels + k];
			}
			spat_diff_image[i - s->init_y][j - s->init_x] = fabs(tot2 - tot1);
		}
	}

	// calculate the mean and standard deviation of the spatial difference image
	spat_diff_img_mean = calc_mean(spat_diff_image, s->init_x, s->init_y, s->final_x, s->final_y);
	spat_diff_img_std = calc_std(spat_diff_image, s->init_x, s->init_y, s->final_x, s->final_y, spat_diff_img_mean);

	// now fill the values of spatial foreground image
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			spatial_foreground[i - s->init_y][j - s->init_x] = (fabs(spat_diff_image[i - s->init_y][j - s->init_x] - spat_diff_img_mean) > (TH_K * spat_diff_img_std)) ? 1 : 0;
		}
	}

	// now fill the final detected foreground
	double foreground_pixel_ratio = 0;
	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			final_detected_foreground[i - s->init_y][j - s->init_x] = temporal_foreground_mask[i - s->init_y][j - s->init_x] && (temporal_foreground[i - s->init_y][j - s->init_x] || spatial_foreground[i - s->init_y][j - s->init_x]);
			if (final_detected_foreground[i - s->init_y][j - s->init_x] == 1)
				foreground_pixel_ratio++;
		}
	}
	foreground_pixel_ratio /= (no_of_pixels_x * no_of_pixels_y);

	// calculate the detected foreground and also the change of this foreground with respect to the previous frame
	double change_detected_foreground = 0;
	double white_to_black = 0;
	double black_to_white = 0;

	for (i = s->init_y; i <= s->final_y; i++)
	{
		for (j = s->init_x; j <= s->final_x; j++)
		{
			if (final_detected_foreground[i - s->init_y][j - s->init_x] == 1)
			{
				if ((det_for_img->imageData + i*det_for_img->widthStep)[j] == 0)
				{
					change_detected_foreground++;
					black_to_white++;
				}
				(det_for_img->imageData + i*det_for_img->widthStep)[j] = 255;
			}
			else
			{
				if ((det_for_img->imageData + i*det_for_img->widthStep)[j] != 0)
				{
					change_detected_foreground++;
					white_to_black++;
				}
				(det_for_img->imageData + i*det_for_img->widthStep)[j] = 0;
			}
		}
	}
	change_detected_foreground /= (no_of_pixels_x * no_of_pixels_y);

	/*
	int sec = (frame_count / 10);	// fps should be replaced
	int min = (sec / 60);
	sec = sec - (min * 60);
	printf("\n frame : %d  time %d:%d ---  current foreground : %lf change foreground : %lf   BTW : %lf   WTB : %lf    BTW / WTB : %lf ", frame_count, min, sec, foreground_pixel_ratio, change_detected_foreground, black_to_white, white_to_black, (black_to_white / white_to_black));
	*/

# if 0
	// show the original image
	cvNamedWindow("Video: original image", CV_WINDOW_AUTOSIZE);
	cvShowImage("Video: original image", middle_frame);
	cvWaitKey(10); //wait for 10 ms for user to hit some key in the window

	// show the detected foreground
	cvNamedWindow("Video: detected foreground", CV_WINDOW_AUTOSIZE);
	cvShowImage("Video: detected foreground", det_for_img);
	cvWaitKey(10); //wait for 10 ms for user to hit some key in the window
# endif

	// motion detection decision
	if ((white_to_black != 0) && (motion_ongoing == 0) &&
			((change_detected_foreground >= 0.05) ||
			 ((black_to_white / white_to_black) >= 1.4) ||
			 	 ((change_detected_foreground >= 0.03) && ((black_to_white / white_to_black) >= 1.1))))
	{
		motion_ongoing = 1;

		/*
		printf("\n --- motion start --- frame : %d  time %d:%d ", frame_count, min, sec);
		printf("\n\t %d:%d ", min, sec);
		*/

		motion_start_frame_count = frame_count;
	}
	else if ((white_to_black != 0) && (motion_ongoing == 1) &&
			(change_detected_foreground < 0.04) && ((change_detected_foreground + (black_to_white / white_to_black)) <= 0.95))
	{
		motion_ongoing = 0;

		/*
		printf(" --- end --- frame : %d  time %d:%d ", frame_count, min, sec);
		printf("\t %d:%d ", min, sec);
		*/

		motion_end_frame_count = frame_count;
		motion_start_time = (motion_start_frame_count / s->tail_video_info->fps);
		motion_end_time = (motion_end_frame_count / s->tail_video_info->fps);

		/*
		 * there should be a new motion information entry
		 * create a motion information node which will be associated with current video file
		 */
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
	}

	// free the allocated structure
	FreeStruct(no_of_pixels_x, no_of_pixels_y);

}	// end function



