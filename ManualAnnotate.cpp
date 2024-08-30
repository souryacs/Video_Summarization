/*
 * ManualAnnotate.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Sourya Bhattacharyya
 */

#include <motion_header.h>

/*
 * this function displays the video segment automatic detected as a motion segment during the optical flow based method,
 * for manual annotation
 */
void DisplayFrameForAnnotate(char* video_filename, char* video_text_out_filename, double motion_start_time,
		double motion_end_time, int start_frame_count, int end_frame_count)
{
	CvCapture* capture = 0;
	int frame_count;
	IplImage* image;
	FILE *fout;
	int label;

# ifdef DISPLAY_HISTORY_ENERGY_IMAGE
    printf("\n motion start : %d  end : %d ", start_frame_count, end_frame_count);
# endif

    capture = cvCaptureFromAVI(video_filename);
    if(capture)
    {
# ifdef DISPLAY_HISTORY_ENERGY_IMAGE
    	cvNamedWindow("Original", 1);
# endif
    	cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, start_frame_count);
        for (frame_count = start_frame_count; frame_count <= end_frame_count; frame_count++)
        {
            image = cvQueryFrame(capture);
            if(!image)
                break;

# ifdef DISPLAY_HISTORY_ENERGY_IMAGE
            cvShowImage("Original", image);
            cvWaitKey(0);	// wait for a key pressed to process to the next frame
# endif
        }	// end for loop of frame by frame processing

        /*
         * now after display of all the frames within the current motion segment
         * prompt to user to give the correct label repeatedly
         * 6 means false motion
         */
    	do
    	{
    		printf("\t Enter label (0 to 6) for motion :  ");
    		scanf("%d", &label);
    	} while ((label < 0) || (label > 6));

		/*
		 * now write the modified label information in the new text file
		 */
    	fout = fopen(video_text_out_filename, "a");
		fprintf(fout, "%lf\t%lf\t%d\t%d\t%d\n", motion_start_time, motion_end_time, start_frame_count, end_frame_count, label);
	    if (fout != NULL)	// close the output file
	    	fclose(fout);

# ifdef DISPLAY_HISTORY_ENERGY_IMAGE
        cvDestroyWindow( "Original" );	// destroy the image window
# endif
        cvReleaseCapture(&capture);	// release the capture of the current video stream
    }	// end capture condition
    return;
}	// end function


/*
 * function to make manual annotation of automatic detected motion segments
 */
void ManualAnnotateSegments(struct motion_detect_basic_struct *s)
{
	int i;
	struct Motion_Information *temp_motion_info;
	struct Video_Information *temp_video_info;
	char vid_file_name_temp[FILENAME_STRING_LEN];	// individual video file name
	char vid_feat_text_out[FILENAME_STRING_LEN];	// the output file to be written for modified annotation
	char temp_vid_idx_str[10];
	FILE *fout;

	// form the output text file which will be used for writing the extracted features and labels
	vid_feat_text_out[0] = '\0';
	strcat(vid_feat_text_out, s->directory_name);
	strcat(vid_feat_text_out, "video_motion_manual_annotation.txt");

	// open the file and close it
	// it will overwrite previous content
	fout = fopen(vid_feat_text_out, "w");
	fclose(fout);

	// initialize the pointer with the header node
	temp_video_info = s->head_video_info;

	while (temp_video_info != NULL)
	{
		// at first form the video file name, to be given as an input to the MHI function
		vid_file_name_temp[0] = '\0';
		strcat(vid_file_name_temp, s->directory_name);
		decimal_to_hex(temp_video_info->video_file_no, &temp_vid_idx_str[0]);
		strcat(vid_file_name_temp, "nrva");
		strcat(vid_file_name_temp, temp_vid_idx_str);
		strcat(vid_file_name_temp, ".avi");
		printf("\n manual annotation for video file : %s  -- motion count : %d  ", vid_file_name_temp, temp_video_info->motion_count);

		// if the current video file contains at least one motion instance
		// then process those motion events
		if (temp_video_info->motion_count >= 1)
		{
			fout = fopen(vid_feat_text_out, "a");
			// write the video related information
			fprintf(fout, "%d", temp_video_info->video_file_no);
			fprintf(fout, "\n%lf", temp_video_info->fps);
			fprintf(fout, "\n%lf", temp_video_info->duration);
			fprintf(fout, "\n%lf", temp_video_info->no_of_frames);
			fprintf(fout, "\n%lf", temp_video_info->frame_height);
			fprintf(fout, "\n%lf", temp_video_info->frame_width);
			fprintf(fout, "\n%d\n", temp_video_info->motion_count);
			fclose(fout);

			// at first, point to the header motion pointer of the video file
			temp_motion_info = temp_video_info->head_motion_info;

			// loop through the motion events
			for (i = 1; i <= temp_video_info->motion_count; i++)
			{
				DisplayFrameForAnnotate(vid_file_name_temp, vid_feat_text_out, temp_motion_info->motion_start_time, temp_motion_info->motion_end_time, temp_motion_info->motion_start_frame_count, temp_motion_info->motion_end_frame_count);

				// advance the motion pointer
				temp_motion_info = temp_motion_info->next;
			}	// end motion count traverse loop
		}	// end if
		temp_video_info = temp_video_info->next;	// advance the video pointer
	}	// end loop for video file traverse
	return;
}	// end function
