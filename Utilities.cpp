/*
 * Utilities.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: sourya Bhattacharyya
 */

#include <motion_header.h>

/*
 * this function converts the decimal no into 4 digit hex for video file name convention
 */
void decimal_to_hex(int num, char *str)
{
	int a[4] = {0,0,0,0};
	int i = 0;

	// convert the number and store it in temporary integer array
	while (num > 15)
	{
	   a[i] = num%16;
	   num = num/16;
	   i++;
	}
	a[i] = num;

	//now store it in the final string
	for (i = 0; i <= 3; i++)
	{
	   if (a[i] == 10)
		   *(str + (3-i)) = 'a';
	   else if (a[i]==11)
		   *(str + (3-i)) = 'b';
	   else if (a[i]==12)
		   *(str + (3-i)) = 'c';
	   else if (a[i]==13)
		   *(str + (3-i)) = 'd';
	   else if (a[i]==14)
		   *(str + (3-i)) = 'e';
	   else if (a[i]==15)
		   *(str + (3-i)) = 'f';
	   else
		   *(str + (3-i)) = ('0' + a[i]);
	}
	*(str + 4) = '\0';	// append a null character

}	// end of decimal to hex conversion method

/*
 * read motion information from the file (if already exists)
 * this file is already written by the static thresholding based motion detection method
*/
void Read_Video_Motion_Information(struct motion_detect_basic_struct *s, char* vid_text_file_name)
{
	FILE *fid;
	int i;
	struct Motion_Information *temp_motion_info;
	struct Video_Information *temp_video_info;
	//int day,mon,yr,hour,min,sec;

	// open the file
	fid = fopen(vid_text_file_name, "r");

	if (fid != NULL)
	{
		// at each step, read information about one video file
		while (!feof(fid))
		{
			// allocate one node of list which will contain information of current video file
			temp_video_info = (struct Video_Information *)malloc(sizeof(struct Video_Information));
			if (temp_video_info == NULL)
			{
				printf("Insufficient memory - no new video node alloc");
				return;
			}

			temp_video_info->next = NULL;
			temp_video_info->prev = NULL;
			if (s->head_video_info == NULL && s->tail_video_info == NULL)	//  this is the first node
			{
				s->head_video_info = s->tail_video_info = temp_video_info;		// assign pointer
			}
			else
			{
				temp_video_info->prev = s->tail_video_info;
				s->tail_video_info->next = temp_video_info;
				s->tail_video_info = temp_video_info;			// assign pointer
			}

			// read individual fields from the text file
			fscanf(fid, "%d", &(s->tail_video_info->video_file_no));

			printf("\n reading motion info for file: %d ", s->tail_video_info->video_file_no);

			//fscanf(fid, "%d", &(s->no_of_video_files));
			//fscanf(fid, "%s", s->tail_video_info->video_filename);
			fscanf(fid, "%lf", &(s->tail_video_info->fps));
			fscanf(fid, "%lf", &(s->tail_video_info->duration));

# if 0
			fprintf(temp_fod, "\n video file : %d  duration : %lf ", no_of_video_files, tail_video_info->duration);
			total_video_duration = total_video_duration + tail_video_info->duration;
# endif

			fscanf(fid, "%lf", &(s->tail_video_info->no_of_frames));
			fscanf(fid, "%lf", &(s->tail_video_info->frame_height));
			fscanf(fid, "%lf", &(s->tail_video_info->frame_width));

# if 0
			// set the recording time structure
			fscanf(fid, "%d %d %d %d %d %d", &day, &mon, &yr, &hour, &min, &sec);
			tail_video_info->video_file_create_time.SetDateTime(yr, mon, day, hour, min, sec);

			// video recording start and end time with respect to raw EEG recording start
			fscanf(fid, "%lf", &(tail_video_info->curr_video_file_start_time));
			fscanf(fid, "%lf", &(tail_video_info->curr_video_file_end_time));
# endif

			// motion information
			fscanf(fid, "%d", &(s->tail_video_info->motion_count));

			printf("\t motions : %d ", s->tail_video_info->motion_count);

			s->tail_video_info->head_motion_info = NULL;
			s->tail_video_info->tail_motion_info = NULL;
			if (s->tail_video_info->motion_count >= 1)
			{
				for (i = 1; i <= s->tail_video_info->motion_count; i++)
				{
					// there is motion information associated with this video file
					// allocate the nodes to store the info
					temp_motion_info = (struct Motion_Information *)malloc(sizeof(struct Motion_Information));
					if (temp_motion_info == NULL)
					{
						printf("Insufficient memory - no new motion node alloc");
						return;
					}

					temp_motion_info->next = NULL;
					temp_motion_info->prev = NULL;
					if (s->tail_video_info->head_motion_info == NULL && s->tail_video_info->tail_motion_info == NULL)	//  this is the first node
					{
						s->tail_video_info->head_motion_info = s->tail_video_info->tail_motion_info = temp_motion_info;		// assign pointer
					}
					else
					{
						temp_motion_info->prev = s->tail_video_info->tail_motion_info;
						s->tail_video_info->tail_motion_info->next = temp_motion_info;
						s->tail_video_info->tail_motion_info = temp_motion_info;			// assign pointer
					}

					// now store the motion information
					fscanf(fid, "%lf", &(s->tail_video_info->tail_motion_info->motion_start_time));
					fscanf(fid, "%lf", &(s->tail_video_info->tail_motion_info->motion_end_time));
					fscanf(fid, "%d", &(s->tail_video_info->tail_motion_info->motion_start_frame_count));
					fscanf(fid, "%d", &(s->tail_video_info->tail_motion_info->motion_end_frame_count));

# ifndef GENERATELABEL
					fscanf(fid, "%d", &(s->tail_video_info->tail_motion_info->motion_type));
# endif


# if 0
					printf("\n motion start time  %lf  end time  %lf  start frame  %d  end frame  %d  label %d ",
							s->tail_video_info->tail_motion_info->motion_start_time, s->tail_video_info->tail_motion_info->motion_end_time,
							s->tail_video_info->tail_motion_info->motion_start_frame_count, s->tail_video_info->tail_motion_info->motion_end_frame_count,
							s->tail_video_info->tail_motion_info->motion_type);
# endif

# if 0
					total_motion_duration = total_motion_duration + (tail_video_info->tail_motion_info->motion_end_time - tail_video_info->tail_motion_info->motion_start_time);
# endif

				}	//end motion info capture loop
			}
		}	// end while loop of file read

# if 0
		fprintf(temp_fod, "\n total video duration : %lf ", total_video_duration);
		fprintf(temp_fod, "\n total motion duration : %lf ", total_motion_duration);
		fclose(temp_fod);
# endif

		// close the file
		fclose(fid);
	}

}



/*
 * this function writes the motion information on a given output file
 * motion info per video file is written
 */
void Write_Motion_Information(struct motion_detect_basic_struct *s, char* vid_text_file_name)
{
	FILE *fod;
	struct Motion_Information *temp_motion_info;
	int i;

	if (s->no_of_video_files == 0)
		fod = fopen(vid_text_file_name, "w");	// this is the first video file - open the file in write mode
	else
	{
		fod = fopen(vid_text_file_name, "a");	// open the file in append text mode
		fprintf(fod, "\n");
	}

	// write the video related information
	fprintf(fod, "%d", s->no_of_video_files);
	//fprintf(fod, "%s\n", tail_video_info->video_filename);
	fprintf(fod, "\n%lf", s->tail_video_info->fps);
	fprintf(fod, "\n%lf", s->tail_video_info->duration);
	fprintf(fod, "\n%lf", s->tail_video_info->no_of_frames);
	fprintf(fod, "\n%lf", s->tail_video_info->frame_height);
	fprintf(fod, "\n%lf", s->tail_video_info->frame_width);
# if 0
	fprintf(fod, "\n%d\t%d\t%d\t%d\t%d\t%d", tail_video_info->video_file_create_time.GetDay(),
				tail_video_info->video_file_create_time.GetMonth(),
				tail_video_info->video_file_create_time.GetYear(),
				tail_video_info->video_file_create_time.GetHour(),
				tail_video_info->video_file_create_time.GetMinute(),
				tail_video_info->video_file_create_time.GetSecond());
	fprintf(fod, "%lf\n", tail_video_info->curr_video_file_start_time);
	fprintf(fod, "%lf\n", tail_video_info->curr_video_file_end_time);
# endif
	fprintf(fod, "\n%d", s->tail_video_info->motion_count);

	// write the motion information in details
	if (s->tail_video_info->motion_count > 0)
	{
		temp_motion_info = s->tail_video_info->head_motion_info;
		for (i = 1; i <= s->tail_video_info->motion_count; i++)
		{
			fprintf(fod, "\n%lf\t%lf\t%d\t%d", temp_motion_info->motion_start_time,
				temp_motion_info->motion_end_time, temp_motion_info->motion_start_frame_count,
				temp_motion_info->motion_end_frame_count);
			temp_motion_info = temp_motion_info->next;	// proceed to the next pointer
		}
	}

	//if (tail_video_info->motion_count >= 1)
	//	fprintf(fod, "\n");

	// insert a backspace character at the end
	// fprintf(fod, "%c", '\b');

	// close the text file
	fclose(fod);

}	// end function

