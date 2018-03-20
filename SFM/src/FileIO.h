#pragma once
#include "io.h"
#include <iostream>
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;
class FilesIO {
private:
	vector<string> FilesName;
	vector<Mat> images;
	bool getFilesName(const string FileDirectory, const string FileType, vector<string>&FilesName)
	{
		string buffer = FileDirectory + "\\*" + FileType;
		_finddata_t c_file;   // 存放文件名的结构体
		intptr_t hFile;
		hFile = _findfirst(buffer.c_str(), &c_file);   //找第一个文件名
		if (hFile == -1L)   // 检查文件夹目录下存在需要查找的文件
			cout << "No" << FileType << "files in current directory!\n" << endl;
		else
		{
			string fullFilePath;
			do
			{
				fullFilePath.clear();
				//名字
				fullFilePath = FileDirectory + c_file.name;
				FilesName.push_back(fullFilePath);
			} while (_findnext(hFile, &c_file) == 0);  //如果找到下个文件的名字成功的话就返回0,否则返回-1  
			_findclose(hFile);
		}
		return FilesName.size() != 0 ? true : false;
	}
	void readImage() {
		for (size_t i = 0;i < FilesName.size(); i++)
		{
			Mat temp = imread(FilesName[i], IMREAD_GRAYSCALE);
			images.push_back(temp);
		}
	}
public:
	FilesIO::FilesIO() {
		cout << "lose parameters（FileDirectory, FileType）" << endl;
		return;
	}
	FilesIO::FilesIO(String FilesDirectory, String FileType)
	{
		if (getFilesName(FilesDirectory, FileType, FilesName))
		{
			readImage();
		}
		else
		{
			cout << "读取失败！" << endl;
		}
	}
	FilesIO::~FilesIO() {
		FilesName.shrink_to_fit();
		images.shrink_to_fit();
	}

	vector<Mat> getImages() {
		return images;
	}
};