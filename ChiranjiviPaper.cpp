/*
 * ChiranjiviPaper.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: sourya Bhattacharyya
 */

#include <motion_header.h>

/*
 * this structure is used to store the cluster centers for each individual pixel
 * the schematic is V[pixel_x][pixel_y][NO_OF_CLUSTER_CENTER]
 */
double*** V;

/*
 * this is the structure containing correlogram for each pixel
 * the schematic of this structure is identified as ---
 * 			[no of pixels in x dir][no of pixels in y dir][no of intensity levels][no of intensity levels]
 */
double**** CorrelogramMat;

/*
 * membership fuzzy matrix for all the pixels combined
 * 3rd index of the membership matrix indicates the cluster center index
 * 4th index signifies the row no (pixel_x) of the particular pixel
 * 5th index indicates the column no (pixel_y) of the particular pixel
 */
double***** membership_mat;

/*
 * this structure holds temporarily the membership matrix for each individual pixel
 */
double*** temp_membershipmat;

/*
 * these structures are used to compute the fuzzy correlogram matrices
 */
double*** reference_fuzzy_correlogram;
double*** current_fuzzy_correlogram;

/*
 * this is a temporary opencv image structure for pixel display
 */
IplImage* display_frame;

/*
 * this function initializes the cluster centers used to compute the fuzzy c means algorithm
 */
void InitializeClusterCenters(double ***V, double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y)
{
	int i;
	int n1, n2;

	/*
	 * initialize each of the cluster centers (corresponding to each of the pixel)
	 * for each pixel, there are l^2 data points where l = no of intensity levels
	 * we initialize first 8 points (8 = no of cluster centers) as the initial value of the cluster centers
	 */
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
			{
				// diagonal matrix elements of the correlogram matrix for each individual pixel are assigned as the cluster centers
				V[n1][n2][i] = CorrelogramMat[n1][n2][i][i];
			}
		}
	}
}

/*
 * this function allocates the cluster centers
 */
void AllocateClusterCenters(double ***V, int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2;

	// allocate cluster centers for each individual pixel
	V = (double***)calloc(no_of_pixels_x, sizeof(double **));
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		V[n1] = (double**)calloc(no_of_pixels_y, sizeof(double *));
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			V[n1][n2] = (double*)calloc(NO_OF_CLUSTER_CENTER, sizeof(double));
		}
	}
}

/*
 * this function frees the cluster centers
 */
void FreeClusterCenters(double ***V, int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2;

	// free cluster centers for each individual pixel
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			free(V[n1][n2]);
		}
		free(V[n1]);
	}
	free(V);
}

/*
 * this function allocates the fuzzy membership matrix mat[cluster][row][column]
 */
void AllocateMembershipMatrix(double***** membership_mat, double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y)
{
	int i, j;
	int n1, n2;

	// allocate the membership matrix
	membership_mat = (double*****)calloc(no_of_pixels_x, sizeof(double ****));
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		membership_mat[n1] = (double****)calloc(no_of_pixels_y, sizeof(double ***));
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			membership_mat[n1][n2] = (double***)calloc(NO_OF_CLUSTER_CENTER, sizeof(double **));
			for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
			{
				membership_mat[n1][n2][i] = (double**)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double *));
				for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
				{
					membership_mat[n1][n2][i][j] = (double*)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double));
				}
			}
		}
	}

	//allocate temp membership matrix
	temp_membershipmat = (double***)calloc(NO_OF_CLUSTER_CENTER, sizeof(double **));
	for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
	{
		temp_membershipmat[i] = (double**)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double *));
		for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
		{
			temp_membershipmat[i][j] = (double*)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double));
		}
	}
}

/*
 * this function frees the fuzzy membership matrix mat[cluster][row][column]
 */
void FreeMembershipMatrix(double***** membership_mat, double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y)
{
	int i, j;
	int n1, n2;

	// free the membership matrix
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
			{
				for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
				{
					free(membership_mat[n1][n2][i][j]);
				}
				free(membership_mat[n1][n2][i]);
			}
			free(membership_mat[n1][n2]);
		}
		free(membership_mat[n1]);
	}
	free(membership_mat);

	//free temp membership matrix
	for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
	{
		for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
		{
			free(temp_membershipmat[i][j]);
		}
		free(temp_membershipmat[i]);
	}
	free(temp_membershipmat);
}

/*
 * this function computes the final membership matrix for each individual pixel input
 */
void ComputeMembershipMatrix(double**** CorrelogramMat, double***** membership_mat, double*** temp_membershipmat,
								int pixel_x_counter, int pixel_y_counter)
{
	int i, j, n1, n2;
	double temp_sum, temp_sum1;
	double pk, vi, vj;
	double error_sum;
	double error_tollerance = 0.01;

	/*
	 * continue the below mentioned loop for calculation of membership matrix and corresponding cluster centers
	 * for each individual input pixel
	 * the loop will continue until the determined membership matrix achieves convergence with respect to a predefined error tolerance value
	 */
	do
	{
		/*
		 * assign the current membership matrix to the temporary one (for the current pixel coordinate)
		 */
		for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
		{
			for (n1 = 0; n1 < TOTAL_INTENSITY_LEVELS; n1++)
			{
				for (n2 = 0; n2 < TOTAL_INTENSITY_LEVELS; n2++)
				{
					temp_membershipmat[i][n1][n2] = membership_mat[pixel_x_counter][pixel_y_counter][i][n1][n2];
				}
			}
		}

		/*
		 * compute the membership matrix element for the current pixel input
		 */
		for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
		{
			vi = V[pixel_x_counter][pixel_y_counter][i];	// initialize the current cluster center
			for (n1 = 0; n1 < TOTAL_INTENSITY_LEVELS; n1++)
			{
				for (n2 = 0; n2 < TOTAL_INTENSITY_LEVELS; n2++)
				{
					// compute the distance from the cluster centers
					temp_sum = 0;
					for (j = 0; j < NO_OF_CLUSTER_CENTER; j++)
					{
						pk = CorrelogramMat[pixel_x_counter][pixel_y_counter][n1][n2];
						vj = V[pixel_x_counter][pixel_y_counter][j];
						temp_sum += (pow(fabs(pk - vi), 2) / pow(fabs(pk - vj), 2));
					}
					membership_mat[pixel_x_counter][pixel_y_counter][i][n1][n2] = (1.0 / temp_sum);
				}
			}
		}

		/*
		 * update the cluster center based on assigned membership matrix
		 * this updation is performed based on individual pixels
		 */
		for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
		{
			temp_sum = 0;
			temp_sum1 = 0;
			for (n1 = 0; n1 < TOTAL_INTENSITY_LEVELS; n1++)
			{
				for (n2 = 0; n2 < TOTAL_INTENSITY_LEVELS; n2++)
				{
					temp_sum += pow(membership_mat[pixel_x_counter][pixel_y_counter][i][n1][n2], 2) * (CorrelogramMat[pixel_x_counter][pixel_y_counter][n1][n2]);
					temp_sum1 += pow(membership_mat[pixel_x_counter][pixel_y_counter][i][n1][n2], 2);
				}
			}
			V[pixel_x_counter][pixel_y_counter][i] = (temp_sum / temp_sum1);
		}

		/*
		 * now check whether new value of membership matrix
		 */
		error_sum = 0;
		for (i = 0; i < NO_OF_CLUSTER_CENTER; i++)
		{
			for (n1 = 0; n1 < TOTAL_INTENSITY_LEVELS; n1++)
			{
				for (n2 = 0; n2 < TOTAL_INTENSITY_LEVELS; n2++)
				{
					error_sum += pow(fabs(membership_mat[pixel_x_counter][pixel_y_counter][i][n1][n2] - temp_membershipmat[i][n1][n2]), 2);
				}
			}
		}
		error_sum = sqrt(error_sum);
	} while(error_sum > error_tollerance);
}

/*
 * this function computes the fuzzy correlogram from the input correlogram matrix
 */
void ComputeFuzzyCorrelogram(double**** CorrelogramMat, double***** membership_mat, double*** fuzzy_corr_mat,
							int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2, c;
	int i, j;
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			for (c = 0; c < NO_OF_CLUSTER_CENTER; c++)
			{
				fuzzy_corr_mat[n1][n2][c] = 0;
				for (i = 0; i < TOTAL_INTENSITY_LEVELS; i++)
				{
					for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
					{
						fuzzy_corr_mat[n1][n2][c] += membership_mat[n1][n2][c][i][j] * CorrelogramMat[n1][n2][i][j];
					}
				}
			}
		}
	}
}

/*
 * this function allocates fuzzy correlogram matrices
 */
void AllocateFuzzyCorrelogramMatrices(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram,
										int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2;

	reference_fuzzy_correlogram = (double***)calloc(no_of_pixels_x, sizeof(double **));
	current_fuzzy_correlogram = (double***)calloc(no_of_pixels_x, sizeof(double **));
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		reference_fuzzy_correlogram[n1] = (double**)calloc(no_of_pixels_y, sizeof(double *));
		current_fuzzy_correlogram[n1] = (double**)calloc(no_of_pixels_y, sizeof(double *));
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			reference_fuzzy_correlogram[n1][n2] = (double*)calloc(NO_OF_CLUSTER_CENTER, sizeof(double));
			current_fuzzy_correlogram[n1][n2] = (double*)calloc(NO_OF_CLUSTER_CENTER, sizeof(double));
		}
	}
}

/*
 * this function frees fuzzy correlogram matrices
 */
void FreeFuzzyCorrelogramMatrices(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram,
									int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2;

	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			free(reference_fuzzy_correlogram[n1][n2]);
			free(current_fuzzy_correlogram[n1][n2]);
		}
		free(reference_fuzzy_correlogram[n1]);
		free(current_fuzzy_correlogram[n1]);
	}
	free(reference_fuzzy_correlogram);
	free(current_fuzzy_correlogram);
}


/*
 * this function allocates the correlogram matrix
 */
void AllocateCorrelogramMatrix(double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y)
{
	int n, i, j;
	CorrelogramMat = (double****)calloc(no_of_pixels_x, sizeof(double ***));
	for (n = 0; n < no_of_pixels_x; n++)
	{
		CorrelogramMat[n] = (double***)calloc(no_of_pixels_y, sizeof(double **));
		for (i = 0; i < no_of_pixels_y; i++)
		{
			CorrelogramMat[n][i] = (double**)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double *));
			for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
			{
				CorrelogramMat[n][i][j] = (double*)calloc(TOTAL_INTENSITY_LEVELS, sizeof(double));
			}
		}	// end pixel y loop
	}	// end pixel x loop
}

/*
 * this function frees the correlogram matrix
 */
void FreeCorrelogramMatrix(double**** CorrelogramMat, int no_of_pixels_x, int no_of_pixels_y)
{
	int n, i, j;
	for (n = 0; n < no_of_pixels_x; n++)
	{
		for (i = 0; i < no_of_pixels_y; i++)
		{
			for (j = 0; j < TOTAL_INTENSITY_LEVELS; j++)
			{
				free(CorrelogramMat[n][i][j]);
			}
			free(CorrelogramMat[n][i]);
		}	// end pixel y loop
		free(CorrelogramMat[n]);
	}	// end pixel x loop
	free(CorrelogramMat);
}

/*
 * calculate correlogram of the input image
 */
void MainCorrelogramCompute(double**** CorrelogramMat, IplImage* cap_frame, int init_x, int final_x, int init_y, int final_y)
{
	int i, j;
	int i1, j1;
	int i2, j2;
	int pix1_ints_class;	// pixel intensity class of 1st pixel in the pair (0 to TOTAL_INTENSITY_LEVELS - 1)
	int pix2_ints_class;	// pixel intensity class of 2nd pixel in the pair (0 to TOTAL_INTENSITY_LEVELS - 1)
	int tot;

	// now within the bounding box, calculate the correlogram for individual pixel
    for(j = init_x; j <= final_x; j++)
	{
		for(i = init_y; i <= final_y; i++)
		{
			// source pixel is (j,i)
			// destination pixel is within 2x2 boundary of this source pixel
			// we will calculate the intensity pair for each such pixel within the distance boundary
			tot = (((cap_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + 0] +
								(cap_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + 1] +
								(cap_frame->imageData + i*cap_frame->widthStep)[j*cap_frame->nChannels + 2]) / 3);
			pix1_ints_class = tot / (MAX_INTENSITY / TOTAL_INTENSITY_LEVELS);

			// now get the target pixel
			for(j1 = j - DISTANCE_PIX; j1 <= j + DISTANCE_PIX; j1++)
			{
				// continue if the region pixel falls outside the marked rectangle
				if ((j1 < init_x) || (j1 > final_x))
					continue;

				for(i1 = i - DISTANCE_PIX; i1 <= i + DISTANCE_PIX; i1++)
				{
					// continue if the region pixel falls outside the marked rectangle
					if ((i1 < init_y) || (i1 > final_y) || ((i1 == i) && (j1 == j)))
						continue;

					// target pixel is (j1, i1)
					// compute its intensity value
					tot = (((cap_frame->imageData + i1*cap_frame->widthStep)[j1*cap_frame->nChannels + 0] +
										(cap_frame->imageData + i1*cap_frame->widthStep)[j1*cap_frame->nChannels + 1] +
										(cap_frame->imageData + i1*cap_frame->widthStep)[j1*cap_frame->nChannels + 2]) / 3);
					pix2_ints_class = tot / (MAX_INTENSITY / TOTAL_INTENSITY_LEVELS);

					// now this intensity value and related correlogram value should be used for neighborhood 5x5 region pixels
					// every such pixel will have corresponding counter updated
					for (j2 = j - REGION_OFFSET; j2 <= j + REGION_OFFSET; j2++)
					{
						// continue if the region pixel falls outside the marked rectangle
						if ((j2 < init_x) || (j2 > final_x))
							continue;
						for (i2 = i - REGION_OFFSET; i2 <= i + REGION_OFFSET; i2++)
						{
							// continue if the region pixel falls outside the marked rectangle
							if ((i2 < init_y) || (i2 > final_y))
								continue;

							// increment the matrix element corresponding to the pixel
							CorrelogramMat[j2 - init_x][i2 - init_y][pix1_ints_class][pix2_ints_class]++;
						}
					}
				}
			}
		}
	}
}

/*
 * this function normalizes the correlogram
 */
void NormalizeCorrelogram(double**** CorrelogramMat, int init_x, int final_x, int init_y, int final_y)
{
	int i, j, i1, j1;
	int curr_pixel_corr_count;

    // now normalize the correlogram
    for(j = init_x; j <= final_x; j++)
	{
		for(i = init_y; i <= final_y; i++)
		{
			curr_pixel_corr_count = 0;
			for (j1 = 0; j1 <= TOTAL_INTENSITY_LEVELS; j1++)
			{
				for (i1  = 0; i1 <= TOTAL_INTENSITY_LEVELS; i1++)
				{
					curr_pixel_corr_count += CorrelogramMat[j - init_x][i - init_y][j1][i1];
				}
			}
			for (j1 = 0; j1 <= TOTAL_INTENSITY_LEVELS; j1++)
			{
				for (i1  = 0; i1 <= TOTAL_INTENSITY_LEVELS; i1++)
				{
					CorrelogramMat[j - init_x][i - init_y][j1][i1] /= curr_pixel_corr_count;
				}
			}
		}
	}
}

/*
 * this function is responsible for allocating all the different structures used in the current computation
 */
void AllocateAllStructures(struct motion_detect_basic_struct *s)
{
	int no_of_pixels_x;
	int no_of_pixels_y;

	no_of_pixels_x = (s->final_x - s->init_x + 1);
	no_of_pixels_y = (s->final_y - s->init_y + 1);

	// at first, for each pixel, allocate the correlogram matrix
	// for no of intensity levels = l, each pixel will contain l^2 * 1 dimension matrix
	AllocateCorrelogramMatrix(CorrelogramMat, no_of_pixels_x, no_of_pixels_y);

	// allocate the fuzzy correlogram matrix and also the temporary matrix
	AllocateMembershipMatrix(membership_mat, temp_membershipmat, no_of_pixels_x, no_of_pixels_y);

	// allocate the cluster centers
	AllocateClusterCenters(V, no_of_pixels_x, no_of_pixels_y);

	// allocate two fuzzy correlogram matrices - one is reference, and another is the current
	// for the first frame, reference is computed
	// from the second frame, current is updated
	AllocateFuzzyCorrelogramMatrices(reference_fuzzy_correlogram, current_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);

	// allocate one image for display of background and foreground pixels
	// the frame has dimension of marked rectangular box
	display_frame = cvCreateImage(cvSize(no_of_pixels_x, no_of_pixels_y),IPL_DEPTH_8U, 1);
}

/*
 * this function frees all the temporary structures
 */
void FreeAllocatedStructures(struct motion_detect_basic_struct *s)
{
	int no_of_pixels_x;
	int no_of_pixels_y;

	no_of_pixels_x = (s->final_x - s->init_x + 1);
	no_of_pixels_y = (s->final_y - s->init_y + 1);

	// free the correlogram matrix
	FreeCorrelogramMatrix(CorrelogramMat, no_of_pixels_x, no_of_pixels_y);

	// free the membership matrices
	FreeMembershipMatrix(membership_mat, temp_membershipmat, no_of_pixels_x, no_of_pixels_y);

	// free the cluster centers
	FreeClusterCenters(V, no_of_pixels_x, no_of_pixels_y);

	// free the fuzzy correlogram matrices
	FreeFuzzyCorrelogramMatrices(reference_fuzzy_correlogram, current_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);

	// free the allocated frame for display of foreground background pixels
	cvReleaseImage(&display_frame);
}

/*
 * this function resets all the allocated structures to 0 for further use
 */
void ResetAllocatedStructures(double ***V, double**** CorrelogramMat, double***** membership_mat,
								double*** temp_membershipmat, int no_of_pixels_x, int no_of_pixels_y)
{
	memset(V, 0, sizeof(V[0][0][0] * no_of_pixels_x * no_of_pixels_y * NO_OF_CLUSTER_CENTER));
	memset(CorrelogramMat, 0, sizeof(CorrelogramMat[0][0][0][0] * no_of_pixels_x * no_of_pixels_y * TOTAL_INTENSITY_LEVELS * TOTAL_INTENSITY_LEVELS));
	memset(membership_mat, 0, sizeof(membership_mat[0][0][0][0][0] * no_of_pixels_x * no_of_pixels_y * NO_OF_CLUSTER_CENTER * TOTAL_INTENSITY_LEVELS * TOTAL_INTENSITY_LEVELS));
	memset(temp_membershipmat, 0, sizeof(temp_membershipmat[0][0][0] * NO_OF_CLUSTER_CENTER * TOTAL_INTENSITY_LEVELS * TOTAL_INTENSITY_LEVELS));
}

/*
 * this function computes the K-L distance between 2 fuzzy correlograms and then determines (based on threshold comparison)
 * which pixel should be considered as a background pixel
 */
void DistanceBasedPixelClassify(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram,
		int no_of_pixels_x, int no_of_pixels_y)
{
	double dist, m, ref, cur;;
	int n1, n2, c;
	double thr = 0.15;

	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			dist = 0;
			for (c = 0; c < NO_OF_CLUSTER_CENTER; c++)
			{
				ref = reference_fuzzy_correlogram[n1][n2][c];
				cur = current_fuzzy_correlogram[n1][n2][c];
				m = 0.5 * (ref + cur);
				dist += cur * log(cur / m) + ref * log(ref / m);
			}

			if (dist < thr)
			{
				// current pixel label = background
				(display_frame->imageData + n2 * no_of_pixels_x)[n1] = 255;
			}
			else
			{
				// current pixel label = foreground
				(display_frame->imageData + n2 * no_of_pixels_x)[n1] = 0;
			}
		}
	}
}

/*
 * fuzzy correlogram update (background and current)
 */
void UpdateFuzzyCorrelogram(double ***reference_fuzzy_correlogram, double ***current_fuzzy_correlogram,
		int no_of_pixels_x, int no_of_pixels_y)
{
	int n1, n2, c;
	double alpha = 0.05;
	for (n1 = 0; n1 < no_of_pixels_x; n1++)
	{
		for (n2 = 0; n2 < no_of_pixels_y; n2++)
		{
			for (c = 0; c < NO_OF_CLUSTER_CENTER; c++)
			{
				reference_fuzzy_correlogram[n1][n2][c] = ((1 - alpha) * reference_fuzzy_correlogram[n1][n2][c] + alpha * current_fuzzy_correlogram[n1][n2][c]);
			}
		}
	}
}

/*
 * this function computes the correlogram for the input frame for individual pixels
 * it calls various functions and integrates the results
 */
void CorrelogramCompute(struct motion_detect_basic_struct *s, IplImage* cap_frame, int frame_count)
{
	int no_of_pixels_x;
	int no_of_pixels_y;
	int pixel_x_counter, pixel_y_counter;

	no_of_pixels_x = (s->final_x - s->init_x + 1);
	no_of_pixels_y = (s->final_y - s->init_y + 1);

	// reset the allocated structures before computation
	// unless the previous values written in those structures can cause incorrect results
	ResetAllocatedStructures(V, CorrelogramMat, membership_mat, temp_membershipmat, no_of_pixels_x, no_of_pixels_y);

	// compute the basic correlogram structure
	MainCorrelogramCompute(CorrelogramMat, cap_frame, s->init_x, s->final_x, s->init_y, s->final_y);

	// normalize correlogram structure
	NormalizeCorrelogram(CorrelogramMat, s->init_x, s->final_x, s->init_y, s->final_y);

	//initialize cluster centers to be used for fuzzy c means algorithm
	InitializeClusterCenters(V, CorrelogramMat, no_of_pixels_x, no_of_pixels_y);

	// now calculate the membership matrix based on fuzzy c means algorithm
	// this is done for each individual pixels
	for (pixel_x_counter = 0; pixel_x_counter < no_of_pixels_x; pixel_x_counter++)
	{
		for (pixel_y_counter = 0; pixel_y_counter < no_of_pixels_y; pixel_y_counter++)
		{
			ComputeMembershipMatrix(CorrelogramMat, membership_mat, temp_membershipmat, pixel_x_counter, pixel_y_counter);
		}
	}

	// now for all the pixels, compute the fuzzy correlogram, which is the product of the membership matrix and the correlogram vector
	// for the first frame, the reference fuzzy correlogram will be computed
	// for subsequent frames, current fuzzy correlogram will be computed
	if (frame_count == 1)
		ComputeFuzzyCorrelogram(CorrelogramMat, membership_mat, reference_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);
	else
		ComputeFuzzyCorrelogram(CorrelogramMat, membership_mat, current_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);

	// now compute the distance between two fuzzy correlogram matrices
	// and set the pixel class (background or foreground accordingly)
	DistanceBasedPixelClassify(reference_fuzzy_correlogram, current_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);

	// update the background fuzzy correlogram matrix from the current one
	UpdateFuzzyCorrelogram(reference_fuzzy_correlogram, current_fuzzy_correlogram, no_of_pixels_x, no_of_pixels_y);
}




