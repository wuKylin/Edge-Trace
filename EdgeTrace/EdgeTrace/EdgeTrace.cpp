// EdgeTrace.cpp : 定义控制台应用程序的入口点。
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void GetBoundryPointIndex(int**m_gray, int height, int width, vector<int> &BPoint)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (m_gray[i][j] != 0)
			{
				if (i == 0 || i == height - 1 || j == 0 || j == width - 1)
				{
					BPoint.push_back(i*width + j);
					continue;
				}
				if (m_gray[i - 1][j] == 0 || m_gray[i - 1][j + 1] == 0 || m_gray[i][j + 1] == 0 || m_gray[i + 1][j + 1] == 0 ||
					m_gray[i + 1][j] == 0 || m_gray[i + 1][j - 1] == 0 || m_gray[i][j - 1] == 0 || m_gray[i - 1][j - 1] == 0)
				{
					//跟踪可加，去掉不必要边界点。 
					int count = 0;
					if (m_gray[i - 1][j] == 0)    count++;
					if (m_gray[i - 1][j + 1] == 0)count++;
					if (m_gray[i][j + 1] == 0)    count++;
					if (m_gray[i + 1][j + 1] == 0)count++;
					if (m_gray[i + 1][j] == 0)    count++;
					if (m_gray[i + 1][j - 1] == 0)count++;
					if (m_gray[i][j - 1] == 0)    count++;
					if (m_gray[i - 1][j - 1] == 0)count++;

					if (count == 1 && (m_gray[i - 1][j + 1] == 0 || m_gray[i + 1][j + 1] == 0 || m_gray[i + 1][j - 1] == 0 || m_gray[i - 1][j - 1] == 0))
						continue;

					BPoint.push_back(i*width + j);

				}
			}
		}
	}
}

void edgeTracing(Mat edgeMat, vector<int>& TPoint, size_t length)//length,num of edge points
{
	int height = edgeMat.rows;
	int width = edgeMat.cols;
	Mat edgeMatPlus1(height + 2, width + 2, CV_8U, Scalar(0));
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			edgeMatPlus1.at<uchar>(i + 1, j + 1) = edgeMat.at<uchar>(i, j);
		}
	}
	int xPos, yPos, xPosPre, yPosPre, xPosPlus, yPosPlus;

	//找到起始跟踪点和第二个跟踪点
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			xPosPlus = i + 1;
			yPosPlus = j + 1;
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus) == 255)
			{
				TPoint.push_back(i*width + j);
				if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
				{
					TPoint.push_back((i + 1)*width + j - 1);
					i = height;
					break;
				}
				else if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus) == 255)
				{
					TPoint.push_back((i + 1)*width + j);
					i = height;
					break;
				}
				else if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
				{
					TPoint.push_back((i + 1)*width + j + 1);
					i = height;
					break;
				}
				else if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
				{
					TPoint.push_back(i*width + j + 1);
					i = height;
					break;
				}
				cerr << " Tracing failed. " << endl;
				break;
			}
		}

	}
	//跟踪
	int TraceIndex = TPoint[1];
	int count = 1;
	do
	{
		xPos = TPoint[count] / width;
		yPos = TPoint[count] % width;
		xPosPlus = xPos + 1;
		yPosPlus = yPos + 1;
		xPosPre = TPoint[count - 1] / width;
		yPosPre = TPoint[count - 1] % width;
		if (xPos - xPosPre == 0 && yPos - yPosPre == 1)
		{
	
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == -1 && yPos - yPosPre == 1)
		{
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus - 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == -1 && yPos - yPosPre == 0)
		{
	
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus - 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == -1 && yPos - yPosPre == -1)
		{
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus - 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>((xPosPlus + 1), yPosPlus) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == 0 && yPos - yPosPre == -1)
		{
	
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus - 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == 1 && yPos - yPosPre == -1)
		{
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus - 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == 1 && yPos - yPosPre == 0)
		{

			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		if (xPos - xPosPre == 1 && yPos - yPosPre == 1)
		{
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus - 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos - 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus + 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos + 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus, yPosPlus + 1) == 255)
			{
				TPoint.push_back(xPos*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus + 1) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos + 1);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
			if (edgeMatPlus1.at<uchar>(xPosPlus - 1, yPosPlus) == 255)
			{
				TPoint.push_back((xPos - 1)*width + yPos);
				count++;
				TraceIndex = TPoint[count];
				continue;
			}
		}
		cerr << "Tracing 2 failed." << endl;

	} while (find(TPoint.begin(), TPoint.end() - 1, TraceIndex) == TPoint.end() - 1 || count<length);
	cout << "Tracing 2 successed." << endl;
}

int main()
{
	string inputDir("srcmat.bmp");
	Mat pSrcImg;
	pSrcImg = imread(inputDir, CV_LOAD_IMAGE_GRAYSCALE);
	int height, width;
	height = pSrcImg.rows;
	width = pSrcImg.cols;
	if (pSrcImg.empty())
	{
		cout << "Could not read the input image:"<<inputDir << endl;
		return -1;
	}
	namedWindow("srcimg", WINDOW_NORMAL);
	imshow("srcimg", pSrcImg);

	int**m_gray;
	m_gray = new int*[pSrcImg.rows];
	for (int i=0; i < height; i++)
	{
		m_gray[i] = new int[width];
		for (int j = 0; j < width; j++)
		{
			m_gray[i][j] = pSrcImg.at<uchar>(i, j);
		}
	}
	vector<int> BPoint;
	GetBoundryPointIndex(m_gray, height, width, BPoint);

	Mat edgeMat(height, width, CV_8U, Scalar(0));
	for (int i = 0; i < BPoint.size(); i++)
	{
		edgeMat.at<uchar>(BPoint[i] / width, BPoint[i] % width) = 255;
	}
	namedWindow("edgeimg", WINDOW_NORMAL);
	imshow("edgeimg", edgeMat);

	vector<int> TPoint;
	edgeTracing(edgeMat, TPoint, BPoint.size());

	Mat traceMat(height, width, CV_8U, Scalar(0));
	for (int i = 0; i < TPoint.size(); i++)
	{
		traceMat.at<uchar>(TPoint[i] / width, TPoint[i] % width) = 255;
	}

	namedWindow("traceimg", WINDOW_NORMAL);
	imshow("traceimg", traceMat);
	waitKey(0);
	
	return 0;
}

