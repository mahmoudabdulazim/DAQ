#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>

class CameraInfo
{
	public:
		cv::FileStorage CalibrationData;
		cv::FileStorage CharacteristicsFile;
		cv::Mat IntrinsicParameters;
		cv::Mat ExtrinsicParameters;
		cv::Mat DistortionCoefficients;
		int ImageWidth;
		int ImageHeight;
		double FoVx;
		double FoVy;
		double ApertureWidth  = 3.67;
		double ApertureHeight = 2.74;
		double FocalLength;
		double AspectRatio;
		cv::Point2d PrincipalPoint;

		void SaveCharacteristics(const std::string& Source)
		{
			CharacteristicsFile = cv::FileStorage(Source,cv::FileStorage::WRITE);

			CharacteristicsFile << "IntrinsicParameters" << IntrinsicParameters;
			CharacteristicsFile << "ExtrinsicParameters" << ExtrinsicParameters;
			CharacteristicsFile << "DistortionCoefficients" << DistortionCoefficients;


			CharacteristicsFile << "AperatureWidth" << ApertureWidth;
			CharacteristicsFile << "AperatureHeight" << ApertureHeight;
			CharacteristicsFile << "FocalLength" << FocalLength;
			CharacteristicsFile << "AspectRatio" << AspectRatio;
			CharacteristicsFile << "PrincipalPoint" << PrincipalPoint;
			CharacteristicsFile << "FoVx" << FoVx;
			CharacteristicsFile << "FoVy" << FoVy;
			CharacteristicsFile << "ImageWidth" << ImageWidth;
			CharacteristicsFile << "ImageHeight" << ImageHeight;

			CharacteristicsFile.release();
		}

		bool ReadCalibrationData(const std::string& Source)
		{
			if (CalibrationData.open(Source,cv::FileStorage::READ))
			{
				CalibrationData["image_width"] >> ImageWidth;
				CalibrationData["image_height"] >> ImageHeight;
				CalibrationData["camera_cv::Matrix"] >> IntrinsicParameters;
				CalibrationData["distortion_coefficients"] >> DistortionCoefficients;
				CalibrationData["extrinsic_parameters"] >> ExtrinsicParameters;


				std::cout << "Extracting Inforcv::Mation from Camera cv::Matrix" << std::endl;
				calibrationMatrixValues(IntrinsicParameters,cv::Size(ImageWidth,ImageHeight),ApertureWidth,ApertureHeight, FoVx, FoVy,FocalLength,PrincipalPoint,AspectRatio);
				std::cout << "Extraction Done, Values Stored" << std::endl;
				return 1;
			}
			else
			{
				std::cout << "Invalid Calibration Data File!" << std::endl;
				return 0;
			}
			CalibrationData.release();
		}

		bool ReadCharacteristics(const std::string& Source)
		{
			if (CharacteristicsFile.open(Source,cv::FileStorage::READ))
			{
				CharacteristicsFile["IntrinsicParameters"] >> IntrinsicParameters;
				CharacteristicsFile["ExtrinsicParameters"] >> ExtrinsicParameters;
				CharacteristicsFile["DistortionCoefficients"] >> DistortionCoefficients;
				CharacteristicsFile["AperatureWidth"] >> ApertureWidth;
				CharacteristicsFile["AperatureHeight"] >> ApertureHeight;
				CharacteristicsFile["AspectRation"] >> AspectRatio;
				CharacteristicsFile["ImageWidth"] >> ImageWidth;
				CharacteristicsFile["ImageHeight"] >> ImageHeight;
				CharacteristicsFile["FoVx"] >> FoVx;
				CharacteristicsFile["FoVy"] >> FoVy;
				CharacteristicsFile["FocalLength"] >> FocalLength;
			}
			else
			{
				std::cout << "Invalid Characteristics File !" << std::endl;
			}
		}
};
