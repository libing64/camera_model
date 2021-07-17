#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "camodocal/chessboard/Chessboard.h"
#include "camodocal/calib/CameraCalibration.h"
#include "camodocal/gpl/gpl.h"

static bool readArucoMarkerParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners,
                                     camodocal::Camera::PatternType patternType)
{
    corners.clear();
    switch(patternType)
    {
    case camodocal::Camera::CHESSBOARD:
    case camodocal::Camera::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
        break;

    case camodocal::Camera::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(cv::Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}

void calcArucoCornerPositions(cv::Ptr<cv::aruco::CharucoBoard>& board, std::vector<int> corners_id, std::vector<cv::Point3f>& objectPoints)
{
    objectPoints.clear();
    for(int i = 0; i < corners_id.size(); i++)
    {
        int id = corners_id[i];
        objectPoints.push_back(board->chessboardCorners[id]);
    }
}

int main(int argc, char** argv)
{
    cv::Size boardSize;
    float squareSize;
    float markerSize;
    int dictionaryId;
    std::string inputDir;
    std::string cameraModel;
    std::string pattern;
    std::string cameraName;
    std::string prefix;
    std::string fileExtension;
    std::string arucoParams;
    bool arucoRefine;
    bool useOpenCV;
    bool viewResults;
    bool verbose;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(8), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(12), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(7.f), "Size of one square in mm")
        ("marker-side-length,ml", boost::program_options::value<float>(&markerSize)->default_value(0.02), "length of aruco side in m")
        ("dictionary-id,d", boost::program_options::value<int>(&dictionaryId)->default_value(0), "aruco marker dictionary id")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("calibrationdata"), "Input directory containing chessboard images")
        ("prefix,p", boost::program_options::value<std::string>(&prefix)->default_value("left-"), "Prefix of images")
        ("pattern",  boost::program_options::value<std::string>(&pattern)->default_value("chessboard"), "Pattern type")
        ("dp",  boost::program_options::value<std::string>(&arucoParams)->default_value(""), "detector parameters")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".png"), "File extension of images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-name", boost::program_options::value<std::string>(&cameraName)->default_value("camera"), "Name of camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(true), "Use OpenCV to detect corners")
        ("rs", boost::program_options::bool_switch(&arucoRefine)->default_value(true), "Refine strategy for aruco detector")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(false), "View results")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(true), "Verbose output")
        ;

    boost::program_options::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(pdesc).run(), vm);
    boost::program_options::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
    {
        std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
        return 1;
    }

    camodocal::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = camodocal::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = camodocal::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = camodocal::Camera::PINHOLE;
    }
    else if (boost::iequals(cameraModel, "scaramuzza"))
    {
        modelType = camodocal::Camera::SCARAMUZZA;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
        return 1;
    }

    switch (modelType)
    {
    case camodocal::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camodocal::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camodocal::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camodocal::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    }

    camodocal::Camera::PatternType patternType = camodocal::Camera::CHESSBOARD;
    if (boost::iequals(pattern, "chessboard"))
    {
        patternType = camodocal::Camera::CHESSBOARD;
    }
    else if (boost::iequals(pattern, "circles_grid"))
    {
        patternType = camodocal::Camera::CIRCLES_GRID;
    }
    else if (boost::iequals(pattern, "asymmetric_circles_grid"))
    {
        patternType = camodocal::Camera::ASYMMETRIC_CIRCLES_GRID;
    }
    else if (boost::iequals(pattern, "aruco"))
    {
        patternType = camodocal::Camera::ARUCO;
    }
    else if (boost::iequals(pattern, "charuco"))
    {
        patternType = camodocal::Camera::CHARUCO;
    } else 
    {
        std::cerr << "# ERROR: Unknown pattern type: " << pattern << std::endl;
        return 1;
    }


    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;

    switch (patternType)
    {
    case camodocal::Camera::CHESSBOARD:
        std::cout << "# INFO: pattern type: chessboard" << std::endl;
        break;
    case camodocal::Camera::CIRCLES_GRID:
        std::cout << "# INFO: pattern type: circles_grid" << std::endl;
        break;
    case camodocal::Camera::ASYMMETRIC_CIRCLES_GRID:
        std::cout << "# INFO: pattern type: asymmetric_circles_grid" << std::endl;
        break;
    case camodocal::Camera::ARUCO:
        std::cout << "# INFO: pattern type: aruco" << std::endl;
        break;
    case camodocal::Camera::CHARUCO:
    {
        std::cout << "# INFO: pattern type: charuco" << std::endl;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

        // create charuco board object
        charucoboard = cv::aruco::CharucoBoard::create(boardSize.width, boardSize.height, squareSize, markerSize, dictionary);
    
        detectorParams = cv::aruco::DetectorParameters::create();
        bool readOk = readArucoMarkerParameters(arucoParams, detectorParams);
        if (!readOk)
        {
            std::cerr << "invalid aruco detector parameters file" << std::endl;
            return 0;
        }
        break;
    }
    default:
        break;
    }

    // look for images in input directory
    std::vector<std::string> imageFilenames;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if prefix matches
        if (!prefix.empty())
        {
            if (filename.compare(0, prefix.length(), prefix) != 0)
            {
                continue;
            }
        }

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        imageFilenames.push_back(itr->path().string());

        if (verbose)
        {
            std::cerr << "# INFO: Adding " << imageFilenames.back() << std::endl;
        }
    }

    if (imageFilenames.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
    }

    cv::Mat image = cv::imread(imageFilenames.front(), -1);
    const cv::Size frameSize = image.size();

    camodocal::CameraCalibration calibration(modelType, cameraName, frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFound(imageFilenames.size(), false);
    for (size_t i = 0; i < imageFilenames.size(); ++i)
    {
        image = cv::imread(imageFilenames.at(i), -1);

        switch (patternType)
        {
            case camodocal::Camera::CHESSBOARD:
            {
                camodocal::Chessboard chessboard(boardSize, image);
                chessboard.findCorners(useOpenCV);
                if (chessboard.cornersFound())
                {
                    if (verbose)
                    {
                        std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
                    }

                    calibration.addChessboardData(chessboard.getCorners());

                    cv::Mat sketch;
                    chessboard.getSketch().copyTo(sketch);

                    cv::imshow("Image", sketch);
                    cv::waitKey(50);
                }
                else if (verbose)
                {
                    std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
                }
                chessboardFound.at(i) = chessboard.cornersFound();
                break;
            }
        case camodocal::Camera::CIRCLES_GRID:
        case camodocal::Camera::ASYMMETRIC_CIRCLES_GRID:
        {
            std::vector<cv::Point2f> circle_points;
            int flags = cv::CALIB_CB_SYMMETRIC_GRID;
            if (patternType == camodocal::Camera::ASYMMETRIC_CIRCLES_GRID)
            {
                flags =  cv::CALIB_CB_ASYMMETRIC_GRID;
            }

            bool found = cv::findCirclesGrid(image, boardSize, circle_points, flags);
            if (found)
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected circles_grid in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
                }
                std::vector<cv::Point3f> objectPoints;
                calcBoardCornerPositions(boardSize, squareSize, objectPoints, patternType);
                calibration.addMarkerData(circle_points, objectPoints);

                cv::Mat sketch;
                image.copyTo(sketch);
                cv::drawChessboardCorners(sketch, boardSize, cv::Mat(circle_points), found);
                cv::imshow("Image", sketch);
                cv::waitKey(50);
            } else if (verbose)
            {
                std::cerr << "# INFO: Did not detect circles_grid in image " << i + 1 << std::endl;
            }
            chessboardFound.at(i) = found;
            break;
        }
        case camodocal::Camera::ARUCO:
        {
            break;
        }
        case camodocal::Camera::CHARUCO:
        {
            std::vector<cv::Point2f> corners, rejected;
            std::vector<int> ids;
            cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);


            // refind strategy to detect more markers
            cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();
            if (arucoRefine)
            {
                cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);
            }
            // interpolate charuco corners
            cv::Mat currentCharucoCorners, currentCharucoIds;
            if(ids.size() > 0)
            {
                cv::aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners,
                                                currentCharucoIds);
            }
            // draw results
            cv::Mat sketch;
            image.copyTo(sketch);
            if(ids.size() > 0) 
            {
                cv::aruco::drawDetectedMarkers(sketch, corners);
            }
            if(currentCharucoCorners.total() > 0)
            {
                cv::aruco::drawDetectedCornersCharuco(sketch, currentCharucoCorners, currentCharucoIds);
            }
            if (ids.size() > 0)
            {
                //get 3d position of aruco corers
                std::vector<cv::Point3f> objectPoints;
                calcArucoCornerPositions(charucoboard, ids, objectPoints);
                calibration.addMarkerData(corners, objectPoints);
                cv::imshow("Image", sketch);
                cv::waitKey(50);
            }
            break;
        }


        default:
            break;
        }
    }

    //     camodocal::Chessboard chessboard(boardSize, image);

    //     chessboard.findCorners(useOpenCV);
    //     if (chessboard.cornersFound())
    //     {
    //         if (verbose)
    //         {
    //             std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
    //         }

    //         calibration.addChessboardData(chessboard.getCorners());

    //         cv::Mat sketch;
    //         chessboard.getSketch().copyTo(sketch);

    //         cv::imshow("Image", sketch);
    //         cv::waitKey(50);
    //     }
    //     else if (verbose)
    //     {
    //         std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
    //     }
    //     chessboardFound.at(i) = chessboard.cornersFound();
    // }
    cv::destroyWindow("Image");

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camodocal::timeInSeconds();

    calibration.calibrate();
    calibration.writeParams(cameraName + "_camera_calib.yaml");
    calibration.writeChessboardData(cameraName + "_chessboard_data.dat");

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                  << " sec.\n";
    }

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration file to " << cameraName + "_camera_calib.yaml" << std::endl;
    }

    if (viewResults)
    {
        std::vector<cv::Mat> cbImages;
        std::vector<std::string> cbImageFilenames;

        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            if (!chessboardFound.at(i))
            {
                continue;
            }

            cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
            cbImageFilenames.push_back(imageFilenames.at(i));
        }

        // visualize observed and reprojected points
        calibration.drawResults(cbImages);

        for (size_t i = 0; i < cbImages.size(); ++i)
        {
            cv::putText(cbImages.at(i), cbImageFilenames.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);
            cv::imshow("Image", cbImages.at(i));
            cv::waitKey(0);
        }
    }

    return 0;
}
