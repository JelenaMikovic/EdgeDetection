#include <iostream>
#include <stdlib.h>
#include "tbb/task_group.h"
#include "tbb/tick_count.h"
#include "BitmapRawConverter.h"

#define __ARG_NUM__				6
#define FILTER_SIZE				3
#define THRESHOLD				128
#define CUTOFF				    500

using namespace std;
using namespace tbb;

// Prewitt operators

int filterHor[FILTER_SIZE * FILTER_SIZE] =
{ -1, 0, 1,
-1, 0, 1,
-1, 0, 1 };
int filterVer[FILTER_SIZE * FILTER_SIZE] =
{ -1, -1, -1,
0, 0, 0, 
1, 1, 1 };
/*
int filterHor[FILTER_SIZE * FILTER_SIZE] =
{ -1,  -1,  0,  1,  1,
-1,  -1,  0,  1,  1,
-1,  -1,  0,  1,  1,
-1,  -1,  0,  1,  1,
-1,  -1,  0,  1,  1 };
int filterVer[FILTER_SIZE * FILTER_SIZE] =
{ -1, -1, -1, -1, -1,
-1, -1, -1, -1, -1,
0, 0,  0, 0, 0,
1, 1, 1, 1, 1,
1, 1, 1, 1, 1 };

int filterHor[FILTER_SIZE * FILTER_SIZE] =
{ -1,  -1,  -1, 0,  1,  1,  1,
-1,  -1,  -1, 0,  1,  1,  1,
-1,  -1,  -1, 0,  1,  1,  1,
-1,  -1,  -1, 0,  1,  1,  1,
-1,  -1,  -1, 0,  1,  1,  1 };
int filterVer[FILTER_SIZE * FILTER_SIZE] =
{ -1, -1, -1, -1, -1, -1, -1,
-1, -1, -1, -1, -1, -1, -1,
0, 0,  0, 0, 0, 0, 0,
1, 1, 1, 1, 1, 1, 1,
1, 1, 1, 1, 1, 1 ,1};*/

/**
* @brief Serial version of edge detection algorithm implementation using Prewitt operator
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void filter_serial_prewitt(int* inBuffer, int* outBuffer, int width, int height) 
{
	for (int i = FILTER_SIZE / 2; i < width - FILTER_SIZE / 2; i++) {
		for (int j = FILTER_SIZE / 2; j < height - FILTER_SIZE / 2; j++) {
			int Gx = 0;
			int Gy = 0;
			int G = 0;
			for (int m = 0; m < FILTER_SIZE * FILTER_SIZE; m++) {
				Gx += inBuffer[(j + filterHor[m]) * width + (i + filterVer[m])] * filterHor[m];
				Gy += inBuffer[(j + filterHor[m]) * width + (i + filterVer[m])] * filterVer[m];
			}

			G = abs(Gx) + abs(Gy);

			if (G > THRESHOLD) {
				outBuffer[j * width + i] = 255;
			}
			else {
				outBuffer[j * width + i] = 0;
			}
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param WIDTH image width
* @param HEIGHT image height
* @param width width of sub matrix
* @param height height of sub matrix
* @param startWidth starting image of sub matrix
* @param startHeight starting height of sub matrix
*/

void parallel_prewitt_cufoff(int* inBuffer, int* outBuffer, int WIDTH, int HEIGHT, int width, int height, int startWidth, int startHeight)
{
	if (startHeight == 0) startHeight += FILTER_SIZE / 2;
	if (startWidth == 0) startWidth += FILTER_SIZE / 2;
	for (int i = startWidth; i < width + startWidth; i++) {
		if (i >= WIDTH - FILTER_SIZE / 2) break;
		for (int j = startHeight; j < height + startHeight; j++) {
			if (j >= HEIGHT - FILTER_SIZE / 2) break;
			int Gx = 0;
			int Gy = 0;
			int G = 0;
			for (int m = 0; m < FILTER_SIZE * FILTER_SIZE; m++) {
				Gx += inBuffer[(j + filterHor[m]) * WIDTH + (i + filterVer[m])] * filterHor[m];
				Gy += inBuffer[(j + filterHor[m]) * WIDTH + (i + filterVer[m])] * filterVer[m];
			}

			G = abs(Gx) + abs(Gy);

			if (G > THRESHOLD) {
				outBuffer[j * WIDTH + i] = 255;
			}
			else {
				outBuffer[j * WIDTH + i] = 0;
			}
		}
	}
}
/**
* @brief Parallel version of edge detection algorithm
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param WIDTH image width
* @param HEIGHT image height
* @param width width of sub matrix
* @param height height of sub matrix
* @param startWidth starting image of sub matrix
* @param startHeight starting height of sub matrix
*/
void filter_parallel_prewitt(int* inBuffer, int* outBuffer, int WIDTH, int HEIGHT, int width, int height, int startWidth, int startHeight)
{
	task_group g;
	if (width < CUTOFF) {
		parallel_prewitt_cufoff(inBuffer, outBuffer, WIDTH, HEIGHT, width, height, startWidth, startHeight);
	}
	else {
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth, startHeight); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth + width / 2, startHeight); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth, startHeight + height / 2); });
		g.run([&] {filter_parallel_prewitt(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth + width / 2, startHeight + height / 2); });
		g.wait();
	}
}

/**
* @brief Making picture black and white
*
* @param inBuffer buffer of input image
* @param width image width
* @param height image height
*/
void black_and_white(int* inBuffer, int width, int height) 
{
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			int value = inBuffer[j * width + i];
			if (value < THRESHOLD) {
				inBuffer[j * width + i] = 0;
			}
			else {
				inBuffer[j * width + i] = 1;
			}
		}
	}
}

/**
* @brief Serial version of edge detection algorithm
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/
void filter_serial_edge_detection(int* inBuffer, int* outBuffer, int width, int height)	
{
	for (int i = FILTER_SIZE / 2; i < width - FILTER_SIZE / 2; i++) {
		for (int j = FILTER_SIZE / 2; j < height - FILTER_SIZE / 2; j++) {
			int P = 0;
			int O = 1;
			int pixel = 0;

			for (int m = 0; m < FILTER_SIZE * FILTER_SIZE; m++) {
				if (m == FILTER_SIZE * FILTER_SIZE/2)
					continue;
				if (inBuffer[(j + filterHor[m]) * width + (i + filterVer[m])] == 1) P = 1;
				if (inBuffer[(j + filterHor[m]) * width + (i + filterVer[m])] == 0) O = 0;
			}

			pixel = abs(P - O);

			if (pixel == 1) {
				outBuffer[j * width + i] = 255;
			}
			else {
				outBuffer[j * width + i] = 0;
			}
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param WIDTH image width
* @param HEIGHT image height
* @param width width of sub matrix
* @param height height of sub matrix
* @param startWidth starting image of sub matrix
* @param startHeight starting height of sub matrix
*/

void parallel_edge_cutoff(int* inBuffer, int* outBuffer, int WIDTH, int HEIGHT, int width, int height, int startWidth, int startHeight)
{
	if (startHeight == 0) startHeight += FILTER_SIZE / 2;
	if (startWidth == 0) startWidth += FILTER_SIZE / 2;
	for (int i = startWidth; i < width + startWidth; i++) {
		if (i >= WIDTH - FILTER_SIZE / 2) break;
		for (int j = startHeight; j < height + startHeight ; j++) {
			if (j >= HEIGHT - FILTER_SIZE / 2) break;
			int P = 0;
			int O = 1;
			int pixel = 0;

			for (int m = 0; m < FILTER_SIZE * FILTER_SIZE; m++) {
				if (m == FILTER_SIZE * FILTER_SIZE / 2)
					continue;
				if (inBuffer[(j + filterHor[m]) * WIDTH + (i + filterVer[m])] == 1) P = 1;
				if (inBuffer[(j + filterHor[m]) * WIDTH + (i + filterVer[m])] == 0) O = 0;
			}

			pixel = abs(P - O);

			if (pixel == 1) {
				outBuffer[j * WIDTH + i] = 255;
			}
			else {
				outBuffer[j * WIDTH + i] = 0;
			}
		}
	}
}

/**
* @brief Parallel version of edge detection algorithm
*
* @param inBuffer buffer of input image
* @param outBuffer buffer of output image
* @param WIDTH image width
* @param HEIGHT image height
* @param width width of sub matrix
* @param height height of sub matrix
* @param startWidth starting image of sub matrix
* @param startHeight starting height of sub matrix
*/
void filter_parallel_edge_detection(int* inBuffer, int* outBuffer, int WIDTH, int HEIGHT, int width, int height, int startWidth, int startHeight)
{
	task_group g;
	if (width < CUTOFF) {
		parallel_edge_cutoff(inBuffer, outBuffer, WIDTH, HEIGHT, width, height, startWidth, startHeight);
	}
	else {
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth, startHeight); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth + width / 2, startHeight); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth, startHeight + height / 2); });
		g.run([&] {filter_parallel_edge_detection(inBuffer, outBuffer, WIDTH, HEIGHT, width / 2, height / 2, startWidth + width / 2, startHeight + height / 2); });
		g.wait();
	}
}

/**
* @brief Function for running test.
*
* @param testNr test identification, 1: for serial version, 2: for parallel version
* @param ioFile input/output file, firstly it's holding buffer from input image and than to hold filtered data
* @param outFileName output file name
* @param outBuffer buffer of output image
* @param width image width
* @param height image height
*/


void run_test_nr(int testNr, BitmapRawConverter* ioFile, char* outFileName, int* outBuffer, unsigned int width, unsigned int height)
{

	// TODO: start measure

	tick_count startTime = tick_count::now();
	int* buffer = ioFile->getBuffer();
		switch (testNr)
		{
		case 1:
			cout << "Running serial version of edge detection using Prewitt operator" << endl;
			filter_serial_prewitt(ioFile->getBuffer(), outBuffer, width, height);
			break;
		case 2:
			cout << "Running parallel version of edge detection using Prewitt operator" << endl;
			filter_parallel_prewitt(ioFile->getBuffer(), outBuffer, width, height, width, height, 0, 0);
			break;
		case 3:
			black_and_white(buffer, width, height);
			cout << "Running serial version of edge detection" << endl;
			filter_serial_edge_detection(ioFile->getBuffer(), outBuffer, width, height);
			break;
		case 4:
			black_and_white(buffer, width, height);
			cout << "Running parallel version of edge detection" << endl;
			filter_parallel_edge_detection(ioFile->getBuffer(), outBuffer, width, height, width, height, 0, 0);
			break;
		default:
			cout << "ERROR: invalid test case, must be 1, 2, 3 or 4!";
			break;
		}
	// TODO: end measure and display time
	tick_count endTime = tick_count::now();
	cout << "\nLasted: " << (endTime - startTime).seconds() * 1000 << "ms." << endl << endl;

	ioFile->setBuffer(outBuffer);
	ioFile->pixelsToBitmap(outFileName);
}

/**
* @brief Print program usage.
*/
void usage()
{
	cout << "\n\ERROR: call program like: " << endl << endl;
	cout << "ProjekatPP.exe";
	cout << " input.bmp";
	cout << " outputSerialPrewitt.bmp";
	cout << " outputParallelPrewitt.bmp";
	cout << " outputSerialEdge.bmp";
	cout << " outputParallelEdge.bmp" << endl << endl;
}

int main(int argc, char* argv[])
{

	if (argc != __ARG_NUM__)
	{
		usage();
		return 0;
	}

	BitmapRawConverter inputFile(argv[1]);
	BitmapRawConverter outputFileSerialPrewitt(argv[1]);
	BitmapRawConverter outputFileParallelPrewitt(argv[1]);
	BitmapRawConverter outputFileSerialEdge(argv[1]);
	BitmapRawConverter outputFileParallelEdge(argv[1]);

	unsigned int width, height;

	int test;

	width = inputFile.getWidth();
	height = inputFile.getHeight();

	int* outBufferSerialPrewitt = new int[width * height];
	int* outBufferParallelPrewitt = new int[width * height];

	memset(outBufferSerialPrewitt, 0x0, width * height * sizeof(int));
	memset(outBufferParallelPrewitt, 0x0, width * height * sizeof(int));

	int* outBufferSerialEdge = new int[width * height];
	int* outBufferParallelEdge = new int[width * height];

	memset(outBufferSerialEdge, 0x0, width * height * sizeof(int));
	memset(outBufferParallelEdge, 0x0, width * height * sizeof(int));

	// serial version Prewitt
	run_test_nr(1, &outputFileSerialPrewitt, argv[2], outBufferSerialPrewitt, width, height);

	// parallel version Prewitt
	run_test_nr(2, &outputFileParallelPrewitt, argv[3], outBufferParallelPrewitt, width, height);

	// serial version special
	run_test_nr(3, &outputFileSerialEdge, argv[4], outBufferSerialEdge, width, height);

	// parallel version special
	run_test_nr(4, &outputFileParallelEdge, argv[5], outBufferParallelEdge, width, height);

	// verification
	cout << "Verification: ";
	test = memcmp(outBufferSerialPrewitt, outBufferParallelPrewitt, width * height * sizeof(int));

	if (test != 0)
	{
		cout << "Prewitt FAIL!" << endl;
	}
	else
	{
		cout << "Prewitt PASS." << endl;
	}

	test = memcmp(outBufferSerialEdge, outBufferParallelEdge, width * height * sizeof(int));

	if (test != 0)
	{
		cout << "Edge detection FAIL!" << endl;
	}
	else
	{
		cout << "Edge detection PASS." << endl;
	}

	// clean up
	delete outBufferSerialPrewitt;
	delete outBufferParallelPrewitt;

	delete outBufferSerialEdge;
	delete outBufferParallelEdge;

	return 0;
}