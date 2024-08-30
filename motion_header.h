/*
 * motion_header.h
 *
 *  Created on: Apr 30, 2012
 *      Author: Sourya Bhattacharyya
 */

#ifndef MOTION_HEADER_H_
#define MOTION_HEADER_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

// file attributes
#include <sys/stat.h>

//opencv function use
#include <cv.h>
#include <highgui.h>
#include <ml.h>

/*
 * any of the following define statements should be enabled
 */
#define PIXEL_DIFFERENCE_MOTION_MEASURE
//#define DYNAMIC_FOREGROUND_MOTION_COMPUTE
//#define CORRELOGRAM_MOTION_MEASURE
//#define MEI_MHI_MOTION_MEASURE
//#define OPTICAL_FLOW_MOTION_VECTOR

/*
 * this define statement can be enabled only when PIXEL_DIFFERENCE_MOTION_MEASURE is enabled
 */
#ifdef PIXEL_DIFFERENCE_MOTION_MEASURE
#define SLEEP_APNEA_DETECT
#endif


/*
 * this define statement is used to enable optical flow based motion vector information
 * within the extracted motion energy segment
 * it can be used in association with MEI_MHI_MOTION_MEASURE
 */
//#define OPT_FLOW_MEI_FEAT_EXTRCT

/*
 * this define statement is to be used only for generating custom labels for
 * if it is used with MEI_MHI_MOTION_MEASURE then it will annotate after genetation of MHI and MEI
 * unless it will be used for generation of manual labels on the motion segments detected by automatic
 * optical flow based motion detection method
 * in that case, this define statement is used for manually updating the label information, without going for any feature extraction
 * or motion energy image computation
 * the input is a text file containing automatic motion detection output (say generated from PIXEL_DIFFERENCE_MOTION_MEASURE)
 * and now the target is to check the annotations and possibly rectify it
 * make sure to off all other define statements above
 */
//#define GENERATELABEL

/*
 * this define statement enables showing the motion history and motion energy image in a frame by frame display
 * it is associated with motion detection using History and Energy image code
 */
//#define DISPLAY_HISTORY_ENERGY_IMAGE



#define FILENAME_STRING_LEN 250
#define TOTAL_INTENSITY_LEVELS 8	// used in intensity bin distribution
#define MAX_INTENSITY 256	// in any image, max intensity value of a pixel
#define REGION_OFFSET 5	// 5x5 region surrounding each pixel
#define DISTANCE_PIX 2	// distance used for correlogram computation
#define NO_OF_CLUSTER_CENTER 8

// thresholds used in paper implementations (Zhang et al.)
#define TH_K 1.6
#define TH_ALPHA 0.1	//0.005

#define NUMBER_OF_FEATURES 400

#define PI 3.14159265358979323846

//#define MAX(a,b) (a>b?a:b)

/*
 * calculating the motion vectors and corresponding angles, we divide the motion vectors into bins of angle 20 degrees each
 */
#define NO_OF_BINS_ANGLE 18

/*
 * this is a structure containing the individual motion vector information
 * the motion vectors are generated due to the execution of the optical flow information
 */
struct MV_inf
{
	int exists;	// boolean variable where 1 means a motion vector really exists
	int startx;
	int endx;
	int starty;
	int endy;
	double len;
	double angle;
};


// structure defining the motion information for each of the video file (if exists) along with recorded EEG data
struct Motion_Information {
	double motion_start_time;
	double motion_end_time;
	int motion_start_frame_count;
	int motion_end_frame_count;
	int motion_type;
	struct Motion_Information *next; // next node pointer
	struct Motion_Information *prev; // prev node pointer
};

// structure containing the information for each video file (if exists) along with recorded EEG data
struct Video_Information {
	int video_file_no;
	double fps;
	double duration;
	double no_of_frames;
	double frame_height;
	double frame_width;
	//char video_filename[20];
	int motion_count;
	//COleDateTime video_file_create_time;
	//double curr_video_file_start_time;	// this is the start time of the current video file with respect to the raw EEG time elapsed
	//double curr_video_file_end_time;		// // this is the end time of the current video file with respect to the raw EEG time elapsed
	struct Motion_Information *head_motion_info;
	struct Motion_Information *tail_motion_info;
	struct Video_Information *next; // next node pointer
	struct Video_Information *prev; // prev node pointer
};

/* global structure used throughout current study */
struct motion_detect_basic_struct
{
	int no_of_video_files; // no of video recorded files
	struct Video_Information *head_video_info; // head pointer to the video information
	struct Video_Information *tail_video_info; // tail pointer to the video information
	char directory_name[FILENAME_STRING_LEN];	// directory of video files
	int init_x, init_y, final_x, final_y;	// bounding rectangle on the video data - to mark the patient
};

/* Defines an enumeration for motion type
 * classifies different type of physiologic or extraphysiologic motion
 */
enum MOTION_TYPE
{
	CAMERA_MOVEMENT,	//0
    NURSE_MOVEMENT,		//1
    PATIENT_TREATMENT,	//2
    RESPIRATION_MOVEMENT,	//3
    BODY_MOVEMENT,	//4
    FACIAL_CHANGES	//5
};

/* function prototypes are declared here */

/*
 * Chiranjivi paper on correlogram compute
 */
void CorrelogramCompute(struct motion_detect_basic_struct *s, IplImage* cap_frame, int frame_count);
void UpdateFuzzyCorrelogram(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram, int no_of_pixels_x, int no_of_pixels_y);
void DistanceBasedPixelClassify(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram, int no_of_pixels_x, int no_of_pixels_y);
void ResetAllocatedStructures(double ***V, double**** CorrelogramMat, double***** membership_mat, double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y);
void FreeAllocatedStructures(struct motion_detect_basic_struct *s);
void AllocateAllStructures(struct motion_detect_basic_struct *s);
void NormalizeCorrelogram(double**** CorrelogramMat, int init_x, int final_x, int init_y, int final_y);
void MainCorrelogramCompute(double**** CorrelogramMat, IplImage* cap_frame, int init_x, int final_x, int init_y, int final_y);
void FreeCorrelogramMatrix(double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y);
void AllocateCorrelogramMatrix(double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y);
void FreeFuzzyCorrelogramMatrices(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram, int no_of_pixels_x, int no_of_pixels_y);
void AllocateFuzzyCorrelogramMatrices(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram, int no_of_pixels_x, int no_of_pixels_y);
void ComputeFuzzyCorrelogram(double**** CorrelogramMat, double***** membership_mat, double*** fuzzy_corr_mat, int no_of_pixels_x, int no_of_pixels_y);
void ComputeMembershipMatrix(double**** CorrelogramMat, double***** membership_mat, double*** temp_membershipmat, int pixel_x_counter, int pixel_y_counter);
void FreeMembershipMatrix(double***** membership_mat, double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y);
void AllocateMembershipMatrix(double***** membership_mat, double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y);
void FreeClusterCenters(double ***V, int no_of_pixels_x, int no_of_pixels_y);
void AllocateClusterCenters(double ***V, int no_of_pixels_x, int no_of_pixels_y);
void InitializeClusterCenters(double ***V, double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y);


/*
 * Motion calculation based on dynamic background computation
 * Zhang et. al.
 */
void HybridMotionDetect(struct motion_detect_basic_struct *s, IplImage* first_frame, IplImage* middle_frame,
						IplImage* last_frame, IplImage* backgrnd_frame, int frame_count, IplImage* det_for_img);
double calc_std(double **img, int init_x, int init_y, int final_x, int final_y, double mean);
double calc_mean(double **img, int init_x, int init_y, int final_x, int final_y);
void AllocateStruct(int no_of_pixels_x, int no_of_pixels_y);
void FreeStruct(int no_of_pixels_x, int no_of_pixels_y);

/*
 * motion calculation based on pixel difference and static threshold based comparison
 * paper used in NCVPRIPG
 */
#ifdef SLEEP_APNEA_DETECT
void motion_detect_static_threshold(struct motion_detect_basic_struct *s, char* video_filename, double Thr_Ratio);
#else
void motion_detect_static_threshold(struct motion_detect_basic_struct *s, char* video_filename);
#endif

/*
 * optical flow computation and motion vector orientation measure
 */
void Optical_Flow_MV_wrapper(struct motion_detect_basic_struct *s);
# ifdef OPT_FLOW_MEI_FEAT_EXTRCT
void Optical_Flow_MV_Compute(char* video_filename, char* video_text_out_filename, int start_frame_count, int end_frame_count, int label, IplImage* motion_energy_img);
# else
void Optical_Flow_MV_Compute(char* video_filename, char* video_text_out_filename, int start_frame_count, int end_frame_count, int label);
# endif
void ComputeEigenHistogram(double pixel_diff_MEI, int total_MV_count_all_frames, double mean_vec_x, double mean_vec_y, int start_frame_count, int end_frame_count,
		struct MV_inf* MV_descr, double* MV_amplitude_mean, int *MV_count, double *MV_amplitude_std, char* video_text_out_filename, int label);
void Init_Hist_Param(double *mean_vec_x, double *mean_vec_y, double* MV_amplitude_mean, int *MV_count, double *MV_amplitude_std);
# ifdef OPT_FLOW_MEI_FEAT_EXTRCT
int CheckMVCompatibility(CvPoint p, CvPoint q, IplImage* motion_energy_img);
# endif
double ConvertAngleRadDeg(double angle_rad);

/*
 * motion history image compute
 */
void Motion_Info_to_History_Img_Wrapper(struct motion_detect_basic_struct *s);
void Motion_History_Image_Compute(char* video_filename, char* video_text_out_filename, int start_frame_count, int end_frame_count, int label);
void MomentFeatExtraction(IplImage* motion_history_img_gray, IplImage* motion_energy_img, char *video_text_out_filename, int label);
void update_mhi(IplImage* img, IplImage* dst, int diff_threshold, bool *significant_motion_state);

/*
 * some utility function - used for all the motion detection methods
 */
void Write_Motion_Information(struct motion_detect_basic_struct *s, char* vid_text_file_name);
void Read_Video_Motion_Information(struct motion_detect_basic_struct *s, char* vid_text_file_name);
void decimal_to_hex(int num, char *str);

/*
 * function to make manual annotation of automatic detected motion segments
 */
void ManualAnnotateSegments(struct motion_detect_basic_struct *s);
void DisplayFrameForAnnotate(char* video_filename, char* video_text_out_filename, double motion_start_time, double motion_end_time, int start_frame_count, int end_frame_count);


#endif /* MOTION_HEADER_H_ */
