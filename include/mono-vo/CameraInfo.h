#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

class CameraInfo
{
	public:
		FileStorage CalibrationData;
		FileStorage CharacteristicsFile;
		Mat IntrinsicParameters;
		Mat ExtrinsicParameters;
		Mat DistortionCoefficients;
		int ImageWidth;
		int ImageHeight;
		double FoVx;
		double FoVy;
		double ApertureWidth  = 3.67;
		double ApertureHeight = 2.74;
		double FocalLength;
		double AspectRatio;
		Point2d PrincipalPoint;

		void SaveCharacteristics(const string& Source)
       	        {
                	CharacteristicsFile = FileStorage(Source,FileStorage::WRITE);

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

                bool ReadCalibrationData(const string& Source)
                {
                        if (CalibrationData.open(Source,FileStorage::READ))
                        {
                                CalibrationData["image_width"] >> ImageWidth;
                                CalibrationData["image_height"] >> ImageHeight;
                                CalibrationData["camera_matrix"] >> IntrinsicParameters;
                                CalibrationData["distortion_coefficients"] >> DistortionCoefficients;
                                CalibrationData["extrinsic_parameters"] >> ExtrinsicParameters;
				
				
                                cout << "Extracting Information from Camera Matrix" << endl;
				calibrationMatrixValues(IntrinsicParameters,Size(ImageWidth,ImageHeight),ApertureWidth,ApertureHeight,FoVx,FoVy,FocalLength,PrincipalPoint,AspectRatio);
				cout << "Extraction Done, Values Stored" << endl;
				return 1;
                        }
                        else
                        {
                                cout << "Invalid Calibration Data File!" << endl;
				return 0;
                        }
			CalibrationData.release();
                }

		bool ReadCharacteristics(const string& Source)
		{
			if (CharacteristicsFile.open(Source,FileStorage::READ))
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
				cout << "Invalid Characteristics File !" << endl;
			}
		}
};
