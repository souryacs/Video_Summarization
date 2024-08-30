/*
 * OpticalMVCompute.cpp
 *
 *  Created on: Aug 4, 2012
 *      Author: sourya
 *      This file contains function for computation and manipulation of optical flow based feature vectors
 *		the motion vectors are used (gradients) for feature based learning
 */

#include <motion_header.h>

static double frame_rate = 10.0;


static CvCapture* capture_flow = 0;
static IplImage* temp_cap_frame_flow = 0;
static IplImage *frame1_1C = 0;
static IplImage *curr_frame = 0;
static IplImage *prev_frame = 0;
static IplImage *frame2_1C = 0;
static IplImage *eig_image = 0;
static IplImage *temp_image = 0;
static IplImage *pyramid1 = 0;
static IplImage *pyramid2 = 0;

/*
static CvPoint2D32f *frame1_features = 0;
static CvPoint2D32f *frame2_features = 0;
static char *optical_flow_found_feature = 0;
float *optical_flow_feature_error = 0;
*/

/* This is just an inline that allocates images.  I did this to reduce clutter in the
 * actual computer vision algorithmic code.  Basically it allocates the requested image
 * unless that image is already non-NULL.  It always leaves a non-NULL image as-is even
 * if that image's size, depth, and/or channels are different than the request.
 */

inline static void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels)
{
	if ( *img != NULL )	return;
	*img = cvCreateImage(size, depth, channels);
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}

/*
 * this function converts the input angle (radian, in the range between -PI to PI to corresponding degrees
 */
double ConvertAngleRadDeg(double angle_rad)
{
	double angle_deg;
	angle_deg = fabs(angle_rad) * 180 / PI;
	if (angle_rad < 0)	// for negative angle
		angle_deg = 360 - angle_deg;
	return angle_deg;
}

/*
 * this function is enabled for simultaneous computation based on motion energy image
 * for any given motion vector (start and end points) it checks whether that belongs to the computed motion energy image
 * if so, then we add the vector in the list for histogram computation
 */

# ifdef OPT_FLOW_MEI_FEAT_EXTRCT
int CheckMVCompatibility(CvPoint p, CvPoint q, IplImage* motion_energy_img)
{
	int mv_exists = 0;
	int i, j;
	int leftx, rightx, topy, bottomy;
	int PIX_OFFSET = 5;

	/*
	 * if the motion vector is zero, then we can ignore it
	 */
	if (((q.x - p.x) == 0) && ((q.y - p.y) == 0))
		return 0;

	/*
	 * check whether the start point exists within the motion energy image
	 * surrounding the point, we draw a 2x2 rectangle and check whether any point belongs to MEI
	 */
	leftx = ((p.x - PIX_OFFSET) > 0) ? (p.x - PIX_OFFSET) : 0;
	rightx = ((p.x + PIX_OFFSET) < motion_energy_img->width) ? (p.x + PIX_OFFSET) : motion_energy_img->width;
	topy = ((p.y - PIX_OFFSET) > 0) ? (p.y - PIX_OFFSET) : 0;
	bottomy = ((p.y + PIX_OFFSET) < motion_energy_img->height) ? (p.y + PIX_OFFSET) : motion_energy_img->height;

    for(j = leftx; j <= rightx; j++)
	{
		for(i = topy; i <= bottomy; i++)
		{
			if ((motion_energy_img->imageData + i*motion_energy_img->widthStep)[j * motion_energy_img->nChannels] != 0)
			{
				mv_exists = 1;
				break;
			}
		}
	}

    if (mv_exists == 0)
    {
    	/*
    	 * check whether the start point exists within the motion energy image
    	 * surrounding the point, we draw a 2x2 rectangle and check whether any point belongs to MEI
    	 */
    	leftx = ((q.x - PIX_OFFSET) > 0) ? (q.x - PIX_OFFSET) : 0;
    	rightx = ((q.x + PIX_OFFSET) < motion_energy_img->width) ? (q.x + PIX_OFFSET) : motion_energy_img->width;
    	topy = ((q.y - PIX_OFFSET) > 0) ? (q.y - PIX_OFFSET) : 0;
    	bottomy = ((q.y + PIX_OFFSET) < motion_energy_img->height) ? (q.y + PIX_OFFSET) : motion_energy_img->height;

        for(j = leftx; j <= rightx; j++)
    	{
    		for(i = topy; i <= bottomy; i++)
    		{
    			if ((motion_energy_img->imageData + i*motion_energy_img->widthStep)[j * motion_energy_img->nChannels] != 0)
    			{
    				mv_exists = 1;
    				break;
    			}
    		}
    	}
    }
	return mv_exists;
}
# endif

/*
 * this function initializes some parameters required for motion vector histogram computation
 */
void Init_Hist_Param(double *mean_vec_x, double *mean_vec_y, double* MV_amplitude_mean, int *MV_count, double *MV_amplitude_std)
{
	int i;

	*mean_vec_x = 0;
	*mean_vec_y = 0;

	for (i = 0; i < NO_OF_BINS_ANGLE; i++)
	{
		*(MV_amplitude_mean + i) = 0;
		*(MV_count + i) = 0;
		*(MV_amplitude_std + i) = 0;
	}
}

/*
 * this function computes the eigenspace of the generated motion vectors
 * it takes the motion vectors and associated structures as the input
 * and form the eigenspace to finally generate the histogram
 */
void ComputeEigenHistogram(double pixel_diff_MEI, int total_MV_count_all_frames, double mean_vec_x, double mean_vec_y, int start_frame_count, int end_frame_count,
		struct MV_inf* MV_descr, double* MV_amplitude_mean, int *MV_count, double *MV_amplitude_std, char* video_text_out_filename, int label)
{
    CvMat *Ma, *Mb, *Mc;
    int i, j, k, index;
	double angle1;
	double hyp1, x;
	FILE *fout;

	// open the output file for feature description
    fout = fopen(video_text_out_filename, "a");

    // first, update the mean vectors
    mean_vec_x /= total_MV_count_all_frames;
    mean_vec_y /= total_MV_count_all_frames;

    /*
     * now first create 2 matrices which will contain all the feature (motion vectors) (one original data and one transpose data)
     * first one will have 2 rows and the no of columns = total_MV_count_all_frames
     * second one will have 2 columns and the no of rows = total_MV_count_all_frames
     */
    Ma = cvCreateMat(2, total_MV_count_all_frames, CV_32FC1);
    Mb = cvCreateMat(total_MV_count_all_frames, 2, CV_32FC1);
    Mc = cvCreateMat(2, 2, CV_32FC1);

    /*
     * now fill the elements of the individual matrices (original and transpose)
     * matrix elements are normalized subject to zero mean condition
     */
    k = 0;
    for (j = start_frame_count; j <= end_frame_count; j++)
    {
    	for (i = 0; i < NUMBER_OF_FEATURES; i++)
    	{
			if (MV_descr[(j - start_frame_count) * NUMBER_OF_FEATURES + i].exists)
			{
				// fill first matrix
				cvmSet(Ma, 0, k, (MV_descr[(j - start_frame_count) * NUMBER_OF_FEATURES + i].endx - MV_descr[(j - start_frame_count) * NUMBER_OF_FEATURES + i].startx - mean_vec_x));
				cvmSet(Ma, 1, k, (MV_descr[(j - start_frame_count) * NUMBER_OF_FEATURES + i].endy - MV_descr[(j - start_frame_count) * NUMBER_OF_FEATURES + i].starty - mean_vec_y));

				// fill second matrix
				cvmSet(Mb, k, 0, cvmGet(Ma, 0, k));
				cvmSet(Mb, k, 1, cvmGet(Ma, 1, k));

				// increment the motion counter
				k++;
			}
    	}
    }

    /*
     * now calculate the eigenvectors and corresponding eigenvalues
     * l = eigenvalues of A (descending order)
     * E = corresponding eigenvectors (rows)
     * Here 2x2 eigenvectors are computed (2 principal components)
     */
    cvMatMul(Ma, Mb, Mc);	// multiply Ma & Mb and store the results in Mc

    CvMat* E  = cvCreateMat(2, 2, CV_32FC1);
    CvMat* l  = cvCreateMat(2, 1, CV_32FC1);
    cvEigenVV(Mc, E, l);

    /*
     * allocate the projection matrix which is the product of the matrix Ma and the eigenmatrix E
     * projection matrix generates the final motion vectors in the direction of eigenspace
     */
    CvMat *Proj_final = cvCreateMat(2, total_MV_count_all_frames, CV_32FC1);

    /*
     * multiply Eigenvector & original data matrices and store the results in the projection matrix
     */
    cvMatMul(E, Ma, Proj_final);

    /*
     * now calculate the magnitude and angle of the individual data (after projection on to the desired eigen space)
     * and also generate the final motion vector histogram
     * cvmGet(Mat, x_index, y_index) gets the value
     * cvmSet(Mat, x_index, y_index, value) sets the value
     */
    for (i = 0; i < total_MV_count_all_frames; i++)
    {
    	angle1 = ConvertAngleRadDeg(atan2((double)cvmGet(Proj_final, 1, i), (double)cvmGet(Proj_final, 0, i)));
    	hyp1 = sqrt(pow((double)cvmGet(Proj_final, 1, i), 2) + pow((double)cvmGet(Proj_final, 0, i), 2));

		/*
		 * now from the derived motion vector (in the eigenspace), update the statistical measures
		 */
		index = (int)(angle1 / (360 / NO_OF_BINS_ANGLE));
		if (index == NO_OF_BINS_ANGLE)
			index--;
		MV_count[index]++;
		MV_amplitude_mean[index] += hyp1;
		MV_amplitude_std[index] += pow(hyp1, 2);
    }

	/*
	 * now finally calculate the statistical features (mean and standard deviation)
	 */
	for (index = 0; index < NO_OF_BINS_ANGLE; index++)
	{
		if (MV_count[index] != 0)
		{
			MV_amplitude_mean[index] /= MV_count[index];

			x = (MV_amplitude_std[index] / MV_count[index]) - pow(MV_amplitude_mean[index], 2);
			if (x >= 0)
				MV_amplitude_std[index] = sqrt(x);
			else
				MV_amplitude_std[index] = 0;
		}
	}

	/*
	 * now write the extracted features and the labels
	 * the features are written for each frame
	 */
	fprintf(fout, "%lf\t%lf\t%d\t%d\t\t", (start_frame_count / frame_rate), (end_frame_count / frame_rate), start_frame_count, end_frame_count);

	fprintf(fout, "%lf\t", ((total_MV_count_all_frames * 1.0) / (end_frame_count - start_frame_count)));	// total MV count (per frame, on avg)
	fprintf(fout, "%lf\t", (pixel_diff_MEI / (end_frame_count - start_frame_count)));	// pixel difference accumulated (per frame, on avg)

	for (index = 0; index < NO_OF_BINS_ANGLE; index++)
	{
		fprintf(fout, "%lf\t", (double)((MV_count[index] * 1.0) / total_MV_count_all_frames));	//MV count proportion
	}
	for (index = 0; index < NO_OF_BINS_ANGLE; index++)
	{
		fprintf(fout, "%lf\t", MV_amplitude_mean[index]);
	}
	for (index = 0; index < NO_OF_BINS_ANGLE; index++)
	{
		fprintf(fout, "%lf\t", MV_amplitude_std[index]);
	}
	fprintf(fout, "%d\n", label);


	/*
	 * now release all the temporary matrices and related data structures
	 */
	cvReleaseMat(&Ma);
	cvReleaseMat(&Mb);
	cvReleaseMat(&Mc);
	cvReleaseMat(&E);
	cvReleaseMat(&l);
	cvReleaseMat(&Proj_final);

    // close the output file
    if (fout != NULL)
    	fclose(fout);

}

/*
 * this function computes the optical flow based motion vectors between two reference frames
 */
#ifdef OPT_FLOW_MEI_FEAT_EXTRCT
void Optical_Flow_MV_Compute(char* video_filename, char* video_text_out_filename, int start_frame_count,
								int end_frame_count, int label, IplImage* motion_energy_img)
# else
void Optical_Flow_MV_Compute(char* video_filename, char* video_text_out_filename, int start_frame_count,
								int end_frame_count, int label)
# endif
{
	struct MV_inf* MV_descr = 0;
	int frame_count;
	int number_of_features = NUMBER_OF_FEATURES;
	int total_MV_count_all_frames = 0;
	double mean_vec_x, mean_vec_y;
	CvSize frame_size;
	CvPoint p,q;
	double angle;
	double hypotenuse;

	double pixel_diff_MEI = 0;

	/* Preparation: This array will contain the features found in frame 1. */
	CvPoint2D32f frame1_features[NUMBER_OF_FEATURES];
	//if (!frame1_features)
	//	frame1_features = (CvPoint2D32f *) calloc(NUMBER_OF_FEATURES, sizeof(CvPoint2D32f));

	/* This array will contain the locations of the points from frame 1 in frame 2. */
	CvPoint2D32f frame2_features[NUMBER_OF_FEATURES];
	//if (!frame2_features)
	//	frame2_features = (CvPoint2D32f *) calloc(NUMBER_OF_FEATURES, sizeof(CvPoint2D32f));

	/* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
	CvSize optical_flow_window = cvSize(3,3);

	/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when epsilon is better than .3.
	 * You can play with these parameters for speed vs. accuracy but these values work pretty well in many situations. */
	CvTermCriteria optical_flow_termination_criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);

	/* The i-th element of this array will be non-zero if and only if the i-th feature of frame 1 was found in frame 2. */
	char optical_flow_found_feature[NUMBER_OF_FEATURES];
	//if (!optical_flow_found_feature)
	//	optical_flow_found_feature = (char *) calloc(NUMBER_OF_FEATURES, sizeof(char));

	/* The i-th element of this array is the error in the optical flow for the i-th feature of curr_frame as found in frame 2.
	 * If the i-th feature was not found (see the array above), I think the i-th entry in this array is undefined. */
	float optical_flow_feature_error[NUMBER_OF_FEATURES];
	//if (!optical_flow_feature_error)
	//	optical_flow_feature_error = (float *) calloc(NUMBER_OF_FEATURES, sizeof(float));

	/*
	 * following arrays store the no of motion vectors falling into respective bins
	 * mean of their respective amplitudes and also the standard deviation
	 */
	double MV_amplitude_mean[NO_OF_BINS_ANGLE];
	int MV_count[NO_OF_BINS_ANGLE];
	double MV_amplitude_std[NO_OF_BINS_ANGLE];
	int i, j, k;

    // capture the video stream
    capture_flow = cvCaptureFromAVI(video_filename);

    if(capture_flow)
    {
        // go to the start frame of the mentioned video file
        cvSetCaptureProperty(capture_flow, CV_CAP_PROP_POS_FRAMES, start_frame_count);
        temp_cap_frame_flow = cvQueryFrame(capture_flow);

    	/*
    	 * allocate the motion vector array dynamically
    	 * there are NO_OF_FEATURES per frame
    	 * multiplying it with the frame count of the current video motion sequence will result in the motion allocation array
    	 */
    	MV_descr = (struct MV_inf*) calloc(((end_frame_count - start_frame_count + 1) * NUMBER_OF_FEATURES), sizeof(struct MV_inf));
    	if (MV_descr == NULL)
    		return;

    	/* Read the video's frame size out of the AVI. */
    	frame_size.height =	temp_cap_frame_flow->height;	// cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_HEIGHT);
    	frame_size.width = temp_cap_frame_flow->width;	//cvGetCaptureProperty(input_video, CV_CAP_PROP_FRAME_WIDTH);

# if 0
		/* Create three windows called "Frame N", "Frame N+1", and "Optical Flow" for visualizing the output.
		 * Have those windows automatically change their size to match the output. */
		cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);

# endif


		allocateOnDemand(&curr_frame, frame_size, IPL_DEPTH_8U, 3);

		/* Allocate another image if not already allocated.
		 * Image has ONE challenge of color (ie: monochrome) with 8-bit "color" depth.
		 * This is the image format OpenCV algorithms actually operate on (mostly).
		 */
		allocateOnDemand(&frame1_1C, frame_size, IPL_DEPTH_8U, 1);
		allocateOnDemand(&frame2_1C, frame_size, IPL_DEPTH_8U, 1);

		/*
		 * we make a full color backup of the captured frame
		 * this is for implementing the frame difference operator between the current frame and the background reference frame
		 */
    	allocateOnDemand(&prev_frame, frame_size, IPL_DEPTH_8U, 3);
    	cvCopyImage(temp_cap_frame_flow, prev_frame);

		/* Convert whatever the AVI image format is into OpenCV's preferred format. AND flip the image vertically.
		 * Flip is a shameless hack.  OpenCV reads in AVIs upside-down by default. ) */
		cvConvertImage(temp_cap_frame_flow, frame1_1C, CV_CVTIMG_FLIP);

    	/*
    	 * before manipulating the motion vectors, lets reset the corresponding structures
    	 */
		Init_Hist_Param(&mean_vec_x, &mean_vec_y, MV_amplitude_mean, MV_count, MV_amplitude_std);

		// init the temporary memory
		allocateOnDemand(&eig_image, frame_size, IPL_DEPTH_32F, 1);
		allocateOnDemand(&temp_image, frame_size, IPL_DEPTH_32F, 1);

		/* This is some workspace for the algorithm.
		 * (The algorithm actually carves the image into pyramids of different resolutions.)
		 */
		allocateOnDemand(&pyramid1, frame_size, IPL_DEPTH_8U, 1);
		allocateOnDemand(&pyramid2, frame_size, IPL_DEPTH_8U, 1);

        /*
         * the following loop will compute the MHI starting from the input starting frame number, upto the end frame number
         * for motion vector compute, we shift the frame by 5 frames, which is for most of the recording video, half of the recorded frame rate
         */
    	for (frame_count = (start_frame_count + 1 /* 5 */); frame_count <= end_frame_count; frame_count += 1 /* 5 */)
        {
    		// Get the next frame of video for comparison (5 frames shift)
    		cvSetCaptureProperty(capture_flow, CV_CAP_PROP_POS_FRAMES, frame_count);
        	temp_cap_frame_flow = cvQueryFrame(capture_flow);

    		/* We'll make a full color backup of this frame so that we can draw on it.
    		 * (It's not the best idea to draw on the static memory space of cvQueryFrame().)
    		 */
        	cvCopyImage(temp_cap_frame_flow, curr_frame);

        	// second image storage for comparison with first image
    		cvConvertImage(temp_cap_frame_flow, frame2_1C, CV_CVTIMG_FLIP);

#ifdef OPT_FLOW_MEI_FEAT_EXTRCT
    		/*
    		 * this portion implements the frame difference operator
    		 * it basically computes the intensity difference between the pixels belonging to the motion energy image (MEI)
    		 */
            for(j = 0; j < motion_energy_img->width; j++)
        	{
        		for(i = 0; i < motion_energy_img->height; i++)
        		{
    				if ((motion_energy_img->imageData + i*motion_energy_img->widthStep)[j*motion_energy_img->nChannels] != 0)
    				{
    	                for(k = 0; k < 3; k++)
    					{
    	                	pixel_diff_MEI += abs((curr_frame->imageData + i*curr_frame->widthStep)[j*curr_frame->nChannels + k]
    								- (prev_frame->imageData + i*prev_frame->widthStep)[j*prev_frame->nChannels + k]);
    	                }
    				}
        		}
        	}
# endif

        	/* Shi and Tomasi Feature Tracking! */

    		// reset the frame 1 feature structure
    		for (i = 0; i < NUMBER_OF_FEATURES; i++)
    		{
    			frame1_features[i].x = 0; frame1_features[i].y = 0;
    		}

        	/* Actually run the Shi and Tomasi algorithm!!
        	 * "frame1_1C" is the input image.
        	 * "eig_image" and "temp_image" are just workspace for the algorithm.
        	 * The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
        	 * The second ".01" specifies the minimum Euclidean distance between features.
        	 * "NULL" means use the entire input image.  You could point to a part of the image.
        	 * WHEN THE ALGORITHM RETURNS:
        	 * "frame1_features" will contain the feature points.
        	 * "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
        	 */
        	cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, .01, NULL);

        	/* Pyramidal Lucas Kanade Optical Flow! */

    		// reset the frame 2 feature structure and also other temporary workspaces
        	for (i = 0; i < NUMBER_OF_FEATURES; i++)
        	{
        		frame2_features[i].x = 0; frame2_features[i].y = 0;
        		optical_flow_found_feature[i] = 0;
        		optical_flow_feature_error[i] = 0;
        	}

        	/* Actually run Pyramidal Lucas Kanade Optical Flow!!
        	 * "frame1_1C" is the first frame with the known features.
        	 * "frame2_1C" is the second frame where we want to find the first frame's features.
        	 * "pyramid1" and "pyramid2" are workspace for the algorithm.
        	 * "frame1_features" are the features from the first frame.
        	 * "frame2_features" is the (output) locations of those features in the second frame.
        	 * "number_of_features" is the number of features in the frame1_features array.
        	 * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
        	 * "5" is the maximum number of pyramids to use.  0 would be just one level.
        	 * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
        	 * "optical_flow_feature_error" is as described above (error in the flow for this feature).
        	 * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
        	 * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
        	 */
        	cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features,
        							number_of_features, optical_flow_window, 0 /* 5 */, optical_flow_found_feature, optical_flow_feature_error,
        							optical_flow_termination_criteria, 0 );

        	/* For fun (and debugging :)), let's draw the flow field. */
        	for(i = 0; i < number_of_features; i++)
        	{
        		/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
        		if (optical_flow_found_feature[i] == 0)
        			continue;

# if 0
        		int line_thickness = 1;

        		/* CV_RGB(red, green, blue) is the red, green, and blue components of the color you want, each out of 255. */
        		CvScalar line_color = CV_RGB(255,0,0);
# endif

        		/* Let's make the flow field look nice with arrows.
        		 * The arrows will be a bit too short for a nice visualization because of the high frame rate
        		 * (i.e.: there's not much motion between the frames).  So let's lengthen them by a factor of 3. */
        		p.x = (int) frame1_features[i].x;
        		p.y = (int) frame1_features[i].y;
        		q.x = (int) frame2_features[i].x;
        		q.y = (int) frame2_features[i].y;

        		angle = atan2((double)p.y - q.y, (double)p.x - q.x);
        		hypotenuse = sqrt(pow((p.y - q.y), 2) + pow((p.x - q.x), 2));

        		/*
        		 * entry the motion vector information (and info of their existence)
        		 * if the motion vector length is 0,0 then we don't consider them in the final list
        		 */
# ifdef OPT_FLOW_MEI_FEAT_EXTRCT
        		if (CheckMVCompatibility(p, q, motion_energy_img))
# else
        		if (((q.x - p.x) != 0) || ((q.y - p.y) != 0))
# endif
        		{
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].startx = p.x;
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].endx = q.x;
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].starty = p.y;
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].endy = q.y;
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].len = hypotenuse;
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].angle = ConvertAngleRadDeg(angle);
					MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].exists = 1;

					// increment the total motion vector
					total_MV_count_all_frames++;

					// compute the mean vector - at first update the partial sum
					mean_vec_x += (q.x - p.x);
					mean_vec_y += (q.y - p.y);

# if 0
					//printf("\t (%d) (%d) <%lf> ", (q.x - p.x), (q.y - p.y), MV_descr[(frame_count - start_frame_count) * NUMBER_OF_FEATURES + i].angle);

					/* Here we lengthen the arrow by a factor of three. */
					q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
					q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

					/* Now we draw the main line of the arrow. */
					/* "curr_frame" is the frame to draw on.
					 * "p" is the point where the line begins.
					 * "q" is the point where the line stops.
					 * "CV_AA" means antialiased drawing.
					 * "0" means no fractional bits in the center cooridinate or radius.
					 */
					cvLine( curr_frame, p, q, line_color, line_thickness, CV_AA, 0 );

					/* Now draw the tips of the arrow.  I do some scaling so that the
					 * tips look proportional to the main line of the arrow. */
					p.x = (int) (q.x + 9 * cos(angle + PI / 4));
					p.y = (int) (q.y + 9 * sin(angle + PI / 4));
					cvLine( curr_frame, p, q, line_color, line_thickness, CV_AA, 0 );
					p.x = (int) (q.x + 9 * cos(angle - PI / 4));
					p.y = (int) (q.y + 9 * sin(angle - PI / 4));
					cvLine( curr_frame, p, q, line_color, line_thickness, CV_AA, 0 );
# endif
        		}	// end condition of motion vector display
        	}	// end feature loop

# if 0
        	/* Now display the image we drew on.  Recall that "Optical Flow" is the name of the window we created above. */
        	printf("\n opt flow -- MV count (cumulative) : %d Displaying image -- %d   ", total_MV_count_all_frames, frame_count);
        	cvShowImage("Optical Flow", curr_frame);
        	cvWaitKey(0);
# endif

        	// now copy the content of the second frame to the first frame
        	// so that the storage can have the latest frame
        	cvCopyImage(frame2_1C, frame1_1C);

        	// copy the content (full color image) of the current frame as the previous frame
        	// it is required for accumulation of the pixel differencing output
        	cvCopyImage(curr_frame, prev_frame);

        }	// end frame by frame loop

# if 0
        cvDestroyWindow("Optical Flow");
# endif

        /***********************************************/
        //printf("\t total MV count : %d ", total_MV_count_all_frames);

        /*
         * this function computes the eigenspace of the generated motion vectors
         * it takes the motion vectors and associated structures as the input
         * and form the eigenspace to finally generate the histogram
         * check --- there is at least one motion vector within the video sequence
         */

        if (total_MV_count_all_frames > 0)
        {
        	ComputeEigenHistogram(pixel_diff_MEI, total_MV_count_all_frames, mean_vec_x, mean_vec_y, start_frame_count, end_frame_count, MV_descr,
        		MV_amplitude_mean, MV_count, MV_amplitude_std, video_text_out_filename, label);
        }
    }	// end capture condition

# if 0
	if (curr_frame)
	{
		cvReleaseImage(&curr_frame);
		curr_frame = 0;
	}
	if (prev_frame)
	{
		cvReleaseImage(&prev_frame);
		prev_frame = 0;
	}
	if (frame1_1C)
	{
		cvReleaseImage(&frame1_1C);
		frame1_1C = 0;
	}
	if (frame2_1C)
	{
		cvReleaseImage(&frame2_1C);
		frame2_1C = 0;
	}
	if (eig_image)
	{
		cvReleaseImage(&eig_image);
		eig_image = 0;
	}
	if (temp_image)
	{
		cvReleaseImage(&temp_image);
		temp_image = 0;
	}
	if (pyramid1)
	{
		cvReleaseImage(&pyramid1);
		pyramid1 = 0;
	}
	if (pyramid2)
	{
		cvReleaseImage(&pyramid2);
		pyramid2 = 0;
	}
# endif


# if 0
	if (frame1_features)
	{
		free(frame1_features);
		frame1_features = 0;
	}
	if (frame2_features)
	{
		free(frame2_features);
		frame2_features = 0;
	}
	if (optical_flow_found_feature)
	{
		free(optical_flow_found_feature);
		optical_flow_found_feature = 0;
	}
	if (optical_flow_feature_error)
	{
		free(optical_flow_feature_error);
		optical_flow_feature_error = 0;
	}
# endif

    /*
     * free the motion vector storage
     */
    //if (MV_descr)
    //{
    	free(MV_descr);
    //}

    // release the capture
    //if (capture_flow)
    //{
    	cvReleaseCapture(&capture_flow);
    //}

    return;
}

/*
 * this function takes the motion instance generated from the previous static thresholding based motion detection algorithm
 * for each such motion instance, it calls the function for generation of the motion vector distribution within the motion sequence
 */
void Optical_Flow_MV_wrapper(struct motion_detect_basic_struct *s)
{
	int i;
	struct Motion_Information *temp_motion_info;
	struct Video_Information *temp_video_info;
	char vid_file_name_temp[FILENAME_STRING_LEN];	// individual video file name
	char vid_feat_text_out[FILENAME_STRING_LEN];	// the output file to be written for feature extraction
	char temp_vid_idx_str[10];

	temp_video_info = s->head_video_info;	// initialize the pointer with the header node

	while (temp_video_info != NULL)
	{
		// at first form the video file name, to be given as an input to the MHI function
		vid_file_name_temp[0] = '\0';
		strcat(vid_file_name_temp, s->directory_name);
		decimal_to_hex(temp_video_info->video_file_no, &temp_vid_idx_str[0]);
		//decimal_to_hex(s->no_of_video_files, &temp_vid_idx_str[0]);
		strcat(vid_file_name_temp, "nrva");
		strcat(vid_file_name_temp, temp_vid_idx_str);
		strcat(vid_file_name_temp, ".avi");

		printf("\n computing motion features for file no : %d ", temp_video_info->video_file_no);

		// form the output text file which will be used for writing the extracted features and labels
		vid_feat_text_out[0] = '\0';
		strcat(vid_feat_text_out, s->directory_name);
		strcat(vid_feat_text_out, "feat_dynamic_backgrnd_box_eig_motion_hist.xls");

		// if the current video file contains at least one motion instance
		// then process those motion events
		if (temp_video_info->motion_count >= 1)
		{
			// at first, point to the header motion pointer of the video file
			temp_motion_info = temp_video_info->head_motion_info;

			// loop through the motion events
			for (i = 1; i <= temp_video_info->motion_count; i++)
			{
# ifndef OPT_FLOW_MEI_FEAT_EXTRCT
				// if the current motion has duration less than 1 sec then we don't consider this motion for learning
				//if ((temp_motion_info->motion_end_frame_count - temp_motion_info->motion_start_frame_count) >= temp_video_info->fps)
				Optical_Flow_MV_Compute(vid_file_name_temp, vid_feat_text_out, temp_motion_info->motion_start_frame_count, temp_motion_info->motion_end_frame_count, temp_motion_info->motion_type);
# endif

				// advance the motion pointer
				temp_motion_info = temp_motion_info->next;
			}
		}
		temp_video_info = temp_video_info->next;	// advance the video pointer
	}	// end of video information process loop
	return;
}

