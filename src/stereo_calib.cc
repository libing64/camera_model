#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "camera_model/chessboard/Chessboard.h"
#include "camera_model/calib/StereoCameraCalibration.h"
#include "camera_model/gpl/gpl.h"

static bool readArucoMarkerParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
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
    //fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
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
void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f> &corners,
                              camera_model::Camera::PatternType patternType)
{
    corners.clear();
    switch (patternType)
    {
    case camera_model::Camera::CHESSBOARD:
    case camera_model::Camera::CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; ++i)
            for (int j = 0; j < boardSize.width; ++j)
                corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        break;

    case camera_model::Camera::ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(cv::Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
        break;
    default:
        break;
    }
}

void calcArucoCornerPositions(cv::Ptr<cv::aruco::CharucoBoard> &board, std::vector<int> corners_id, std::vector<cv::Point3f> &objectPoints)
{
    objectPoints.clear();
    for (int i = 0; i < corners_id.size(); i++)
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
    std::string outputDir;
    std::string cameraModel;
    std::string pattern;
    std::string cameraNameL, cameraNameR;
    std::string prefixL, prefixR;
    std::string fileExtension;
    std::string arucoParams;
    bool useOpenCV;
    bool viewResults;
    bool verbose;

    //========= Handling Program options =========
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("width,w", boost::program_options::value<int>(&boardSize.width)->default_value(9), "Number of inner corners on the chessboard pattern in x direction")
        ("height,h", boost::program_options::value<int>(&boardSize.height)->default_value(6), "Number of inner corners on the chessboard pattern in y direction")
        ("size,s", boost::program_options::value<float>(&squareSize)->default_value(120.f), "Size of one square in mm")
        ("marker-size,ms", boost::program_options::value<float>(&markerSize)->default_value(0.02), "length of aruco side in m")
        ("dictionary-id,d", boost::program_options::value<int>(&dictionaryId)->default_value(0), "aruco marker dictionary id")
        ("input,i", boost::program_options::value<std::string>(&inputDir)->default_value("images"), "Input directory containing chessboard images")
        ("output,o", boost::program_options::value<std::string>(&outputDir)->default_value("."), "Output directory containing calibration data")
        ("prefix-l", boost::program_options::value<std::string>(&prefixL)->default_value("left"), "Prefix of images from left camera")
        ("prefix-r", boost::program_options::value<std::string>(&prefixR)->default_value("right"), "Prefix of images from right camera")
        ("pattern",  boost::program_options::value<std::string>(&pattern)->default_value("chessboard"), "Pattern type")
        ("dp",  boost::program_options::value<std::string>(&arucoParams)->default_value(""), "detector parameters")
        ("file-extension,e", boost::program_options::value<std::string>(&fileExtension)->default_value(".bmp"), "File extension of images")
        ("camera-model", boost::program_options::value<std::string>(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | mei | pinhole")
        ("camera-name-l", boost::program_options::value<std::string>(&cameraNameL)->default_value("camera_left"), "Name of left camera")
        ("camera-name-r", boost::program_options::value<std::string>(&cameraNameR)->default_value("camera_right"), "Name of right camera")
        ("opencv", boost::program_options::bool_switch(&useOpenCV)->default_value(false), "Use OpenCV to detect corners")
        ("view-results", boost::program_options::bool_switch(&viewResults)->default_value(false), "View results")
        ("verbose,v", boost::program_options::bool_switch(&verbose)->default_value(false), "Verbose output")
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

    camera_model::Camera::ModelType modelType;
    if (boost::iequals(cameraModel, "kannala-brandt"))
    {
        modelType = camera_model::Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(cameraModel, "mei"))
    {
        modelType = camera_model::Camera::MEI;
    }
    else if (boost::iequals(cameraModel, "pinhole"))
    {
        modelType = camera_model::Camera::PINHOLE;
    }
    else if (boost::iequals(cameraModel, "scaramuzza"))
    {
        modelType = camera_model::Camera::SCARAMUZZA;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
        return 1;
    }

    switch (modelType)
    {
    case camera_model::Camera::KANNALA_BRANDT:
        std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
        break;
    case camera_model::Camera::MEI:
        std::cout << "# INFO: Camera model: Mei" << std::endl;
        break;
    case camera_model::Camera::PINHOLE:
        std::cout << "# INFO: Camera model: Pinhole" << std::endl;
        break;
    case camera_model::Camera::SCARAMUZZA:
        std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
        break;
    }

    camera_model::Camera::PatternType patternType = camera_model::Camera::CHESSBOARD;
    if (boost::iequals(pattern, "chessboard"))
    {
        patternType = camera_model::Camera::CHESSBOARD;
    }
    else if (boost::iequals(pattern, "circles_grid"))
    {
        patternType = camera_model::Camera::CIRCLES_GRID;
    }
    else if (boost::iequals(pattern, "asymmetric_circles_grid"))
    {
        patternType = camera_model::Camera::ASYMMETRIC_CIRCLES_GRID;
    }
    else if (boost::iequals(pattern, "aruco"))
    {
        patternType = camera_model::Camera::ARUCO;
    }
    else if (boost::iequals(pattern, "charuco"))
    {
        patternType = camera_model::Camera::CHARUCO;
    }
    else
    {
        std::cerr << "# ERROR: Unknown pattern type: " << pattern << std::endl;
        return 1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;

    switch (patternType)
    {
    case camera_model::Camera::CHESSBOARD:
        std::cout << "# INFO: pattern type: chessboard" << std::endl;
        break;
    case camera_model::Camera::CIRCLES_GRID:
        std::cout << "# INFO: pattern type: circles_grid" << std::endl;
        break;
    case camera_model::Camera::ASYMMETRIC_CIRCLES_GRID:
        std::cout << "# INFO: pattern type: asymmetric_circles_grid" << std::endl;
        break;
    case camera_model::Camera::ARUCO:
        std::cout << "# INFO: pattern type: aruco" << std::endl;
        break;
    case camera_model::Camera::CHARUCO:
    {
        std::cout << "# INFO: pattern type: charuco" << std::endl;
        std::cout << "# INFO: dictionary id: " << dictionaryId << std::endl;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

        // create charuco board object
        charucoboard = cv::aruco::CharucoBoard::create(boardSize.width + 1, boardSize.height + 1, squareSize, markerSize, dictionary);
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
    std::vector<std::string> imageFilenamesL, imageFilenamesR;
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
        {
            continue;
        }

        std::string filename = itr->path().filename().string();

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
        {
            continue;
        }

        // check if prefix matches
        if (prefixL.empty() || (!prefixL.empty() && (filename.compare(0, prefixL.length(), prefixL) == 0)))
        {
            imageFilenamesL.push_back(itr->path().string());

            if (verbose)
            {
                std::cerr << "# INFO: Adding " << imageFilenamesL.back() << std::endl;
            }
        }
        if (prefixR.empty() || (!prefixR.empty() && (filename.compare(0, prefixR.length(), prefixR) == 0)))
        {
            imageFilenamesR.push_back(itr->path().string());

            if (verbose)
            {
                std::cerr << "# INFO: Adding " << imageFilenamesR.back() << std::endl;
            }
        }
    }

    if (imageFilenamesL.empty() || imageFilenamesR.empty())
    {
        std::cerr << "# ERROR: No chessboard images found." << std::endl;
        return 1;
    }

    if (imageFilenamesL.size() != imageFilenamesR.size())
    {
        std::cerr << "# ERROR: # chessboard images from left and right cameras do not match." << std::endl;
        return 1;
    }

    bool matchImages = true;
    std::sort(imageFilenamesL.begin(), imageFilenamesL.end());
    std::sort(imageFilenamesR.begin(), imageFilenamesR.end());

    for (size_t i = 0; i < imageFilenamesL.size(); ++i)
    {
        std::string filenameL = boost::filesystem::path(imageFilenamesL.at(i)).filename().string();
        std::string filenameR = boost::filesystem::path(imageFilenamesR.at(i)).filename().string();

        if (filenameL.compare(prefixL.length(),
                              filenameL.size() - prefixL.length(),
                              filenameR,
                              prefixR.length(),
                              filenameR.size() - prefixR.length()) != 0)
        {
            matchImages = false;

            if (verbose)
            {
                std::cerr << "# ERROR: Filenames do not match: "
                          << imageFilenamesL.at(i) << " " << imageFilenamesR.at(i) << std::endl;
            }
        }
    }

    if (!matchImages)
    {
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: # images: " << imageFilenamesL.size() << std::endl;
    }

    cv::Mat imageL = cv::imread(imageFilenamesL.front(), -1);
    cv::Mat imageR;
    const cv::Size frameSize = imageL.size();

    camera_model::StereoCameraCalibration calibration(modelType, cameraNameL, cameraNameR, frameSize, boardSize, squareSize);
    calibration.setVerbose(verbose);

    std::vector<bool> chessboardFoundL(imageFilenamesL.size(), false);
    std::vector<bool> chessboardFoundR(imageFilenamesR.size(), false);
    for (size_t i = 0; i < imageFilenamesL.size(); ++i)
    {
        imageL = cv::imread(imageFilenamesL.at(i), -1);
        imageR = cv::imread(imageFilenamesR.at(i), -1);

        switch (patternType)
        {
        case camera_model::Camera::CHESSBOARD:
        {
            camera_model::Chessboard chessboardL(boardSize, imageL);
            camera_model::Chessboard chessboardR(boardSize, imageR);

            chessboardL.findCorners(useOpenCV);
            chessboardR.findCorners(useOpenCV);
            if (chessboardL.cornersFound() && chessboardR.cornersFound())
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected chessboard in image " << i + 1 << std::endl;
                }

                calibration.addChessboardData(chessboardL.getCorners(),
                                              chessboardR.getCorners());

                cv::Mat sketch;
                chessboardL.getSketch().copyTo(sketch);

                cv::imshow("Image - Left", sketch);

                chessboardR.getSketch().copyTo(sketch);

                cv::imshow("Image - Right", sketch);

                cv::waitKey(50);
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
            }
            chessboardFoundL.at(i) = chessboardL.cornersFound();
            chessboardFoundR.at(i) = chessboardR.cornersFound();
            break;
        }
        case camera_model::Camera::CIRCLES_GRID:
        case camera_model::Camera::ASYMMETRIC_CIRCLES_GRID:
        {
            std::vector<cv::Point2f> circle_pointsL, circle_pointsR;
            int flags = cv::CALIB_CB_SYMMETRIC_GRID;
            if (patternType == camera_model::Camera::ASYMMETRIC_CIRCLES_GRID)
            {
                flags = cv::CALIB_CB_ASYMMETRIC_GRID;
            }

            bool foundL = cv::findCirclesGrid(imageL, boardSize, circle_pointsL, flags);
            bool foundR = cv::findCirclesGrid(imageR, boardSize, circle_pointsR, flags);
            if (foundL && foundR)
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected circles_grid in image " << i + 1 << std::endl;
                }
                std::vector<cv::Point3f> objectPoints;
                calcBoardCornerPositions(boardSize, squareSize, objectPoints, patternType);
                calibration.addCornersData(circle_pointsL, circle_pointsR, objectPoints);

                cv::Mat sketch;
                imageL.copyTo(sketch);
                cv::drawChessboardCorners(sketch, boardSize, cv::Mat(circle_pointsL), foundL);
                cv::imshow("Image - Left", sketch);

                imageR.copyTo(sketch);
                cv::drawChessboardCorners(sketch, boardSize, cv::Mat(circle_pointsR), foundR);
                cv::imshow("Image - Right", sketch);
                
                cv::waitKey(50);
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect circles_grid in image " << i + 1 << std::endl;
            }
            chessboardFoundL.at(i) = foundL;
            chessboardFoundR.at(i) = foundR;
            break;
        }
        case camera_model::Camera::ARUCO:
        {
            break;
        }
        case camera_model::Camera::CHARUCO:
        {
            std::vector<std::vector<cv::Point2f>> cornersL, cornersR, rejectedL, rejectedR;
            std::vector<int> idsL, idsR;
            //detect aruco markers [aruco_cnt * 4]
            cv::aruco::detectMarkers(imageL, dictionary, cornersL, idsL, detectorParams, rejectedL);
            cv::aruco::detectMarkers(imageR, dictionary, cornersR, idsR, detectorParams, rejectedR);

            // refind strategy to detect more markers
            cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();
            bool arucoRefine = true;
            if (arucoRefine)
            {
                cv::aruco::refineDetectedMarkers(imageL, board, cornersL, idsL, rejectedL);
            }
            // interpolate corners
            std::vector<cv::Point2f> charuco_cornersL, charuco_cornersR;
            std::vector<int> charuco_idsL, charuco_idsR;
            if (idsL.size() > 0 && idsR.size() > 0)
            {
                cv::aruco::interpolateCornersCharuco(cornersL, idsL, imageL, charucoboard, charuco_cornersL,
                                                     charuco_idsL);
                cv::aruco::interpolateCornersCharuco(cornersR, idsR, imageR, charucoboard, charuco_cornersR,
                                                     charuco_idsR);
            }
            // draw results
            cv::Mat sketch;
            imageL.copyTo(sketch);
            if (idsL.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(sketch, cornersL);
            }
            if (charuco_cornersL.size() > 0)
            {
                cv::aruco::drawDetectedCornersCharuco(sketch, charuco_cornersL, charuco_idsL);
            }
            //todo, judge ids for left and right
            if (charuco_idsL.size() == boardSize.width * boardSize.height && charuco_idsL.size() == charuco_idsR.size())
            {
                chessboardFoundL.at(i) = true;
                chessboardFoundR.at(i) = true;
                //get 3d position of aruco corers
                std::vector<cv::Point3f> objectPoints;
                calcArucoCornerPositions(charucoboard, charuco_idsL, objectPoints);

                calibration.addCornersData(charuco_cornersL, charuco_cornersR, objectPoints);

                cv::imshow("Image - Left", sketch);
                cv::imshow("Image - Right", sketch);//todo show right image

                cv::waitKey(50);
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect charuco in image " << i + 1 << std::endl;
            }
            break;
        }
        default:
            break;
        }
        
    }
    cv::destroyWindow("Image - Left");
    cv::destroyWindow("Image - Right");

    if (calibration.sampleCount() < 10)
    {
        std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
        return 1;
    }

    if (verbose)
    {
        std::cerr << "# INFO: Calibrating..." << std::endl;
    }

    double startTime = camera_model::timeInSeconds();

    calibration.calibrate();
    calibration.writeParams(outputDir);

    if (verbose)
    {
        std::cout << "# INFO: Calibration took a total time of "
                  << std::fixed << std::setprecision(3) << camera_model::timeInSeconds() - startTime
                  << " sec.\n";
    }

    if (verbose)
    {
        std::cerr << "# INFO: Wrote calibration files to " << boost::filesystem::absolute(outputDir).string() << std::endl;
    }

    if (viewResults)
    {
        std::vector<cv::Mat> cbImagesL, cbImagesR;
        std::vector<std::string> cbImageFilenamesL;
        std::vector<std::string> cbImageFilenamesR;

        for (size_t i = 0; i < imageFilenamesL.size(); ++i)
        {
            if (!chessboardFoundL.at(i) || !chessboardFoundR.at(i))
            {
                continue;
            }

            cbImagesL.push_back(cv::imread(imageFilenamesL.at(i), -1));
            cbImageFilenamesL.push_back(imageFilenamesL.at(i));

            cbImagesR.push_back(cv::imread(imageFilenamesR.at(i), -1));
            cbImageFilenamesR.push_back(imageFilenamesR.at(i));
        }

        // visualize observed and reprojected points
        calibration.drawResults(cbImagesL, cbImagesR);

        for (size_t i = 0; i < cbImagesL.size(); ++i)
        {
            cv::putText(cbImagesL.at(i), cbImageFilenamesL.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);
            cv::imshow("Image - Left", cbImagesL.at(i));
            cv::putText(cbImagesR.at(i), cbImageFilenamesR.at(i), cv::Point(10,20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                        1, CV_AA);
            cv::imshow("Image - Right", cbImagesR.at(i));
            cv::waitKey(0);
        }
    }

    return 0;
}
